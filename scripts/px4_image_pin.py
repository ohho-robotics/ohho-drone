#!/usr/bin/env python3
"""Bind the pinned px4io/px4-dev digests to the image id SITL runs.

``docker save`` does not keep ``RepoDigests``. A tarball from the actions
cache still has the image id, which is the config digest of the amd64
manifest. This fetches the index and amd64 manifests by the pinned digests,
hashes the raw bytes, and requires:

- the index lists the pinned amd64 manifest for linux/amd64
- that manifest's config digest is ``PX4_IMAGE_ID``

The workflow then refuses any other local image id.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Mapping, Optional, Sequence

INDEX_ACCEPT = (
    "application/vnd.docker.distribution.manifest.list.v2+json, "
    "application/vnd.oci.image.index.v1+json"
)
MANIFEST_ACCEPT = (
    "application/vnd.docker.distribution.manifest.v2+json, "
    "application/vnd.oci.image.manifest.v1+json"
)
_REPO = re.compile(r"[a-z0-9]+(?:[._-][a-z0-9]+)*/[a-z0-9]+(?:[._-][a-z0-9]+)*")
_DIGEST = re.compile(r"sha256:[0-9a-f]{64}")
_MAX_BYTES = 1024 * 1024
_USER_AGENT = "ohho-drone-px4-image-pin"


class PinError(RuntimeError):
    """The image is not the pinned v1.17.0 image."""


def check_repo(repo: str) -> str:
    if not _REPO.fullmatch(repo):
        raise PinError("refusing image: unexpected repository name")
    return repo


def check_digest(value: str, label: str) -> str:
    if not _DIGEST.fullmatch(value):
        raise PinError(f"refusing image: {label} is not a sha256 digest")
    return value


def sha256_digest(body: bytes) -> str:
    return "sha256:" + hashlib.sha256(body).hexdigest()


def parse_verified_manifest(body: bytes, expected_digest: str) -> dict:
    """Return the manifest object when ``body`` hashes to ``expected_digest``."""
    check_digest(expected_digest, "manifest digest")
    actual = sha256_digest(body)
    if actual != expected_digest:
        raise PinError(
            f"refusing image: manifest hashed to {actual}, not {expected_digest}"
        )
    try:
        parsed = json.loads(body)
    except json.JSONDecodeError as exc:
        raise PinError(
            f"refusing image: manifest {expected_digest} is not json"
        ) from exc
    if not isinstance(parsed, dict):
        raise PinError(f"refusing image: manifest {expected_digest} is not an object")
    return parsed


def linux_amd64_digests(index: Mapping[str, object]) -> list[str]:
    found: list[str] = []
    manifests = index.get("manifests")
    if not isinstance(manifests, list):
        return found
    for item in manifests:
        if not isinstance(item, dict):
            continue
        platform = item.get("platform")
        if not isinstance(platform, dict):
            continue
        if platform.get("os") != "linux" or platform.get("architecture") != "amd64":
            continue
        digest = item.get("digest")
        if isinstance(digest, str):
            found.append(digest)
    return found


def verified_image_id(
    index_body: bytes,
    index_digest: str,
    amd64_body: bytes,
    amd64_digest: str,
    image_id: str,
) -> str:
    """Return ``image_id`` when both manifests hash to the pinned digests."""
    check_digest(image_id, "image id")
    index = parse_verified_manifest(index_body, index_digest)
    amd64 = parse_verified_manifest(amd64_body, amd64_digest)
    if amd64_digest not in linux_amd64_digests(index):
        raise PinError(
            "refusing image: index digest does not list the pinned amd64 image"
        )
    config = amd64.get("config")
    if not isinstance(config, dict):
        raise PinError("refusing image: pinned amd64 manifest has no config")
    config_digest = config.get("digest")
    if config_digest != image_id:
        raise PinError(
            f"refusing image: config digest {config_digest} is not the pinned image id"
        )
    layers = amd64.get("layers")
    if not isinstance(layers, list) or not layers:
        raise PinError("refusing image: pinned amd64 manifest has no layers")
    for layer in layers:
        digest = layer.get("digest") if isinstance(layer, dict) else None
        if not isinstance(digest, str) or not _DIGEST.fullmatch(digest):
            raise PinError(
                "refusing image: pinned amd64 manifest has a layer without a digest"
            )
    return image_id


def _request_bytes(url: str, headers: dict[str, str], timeout: float, what: str) -> bytes:
    delay = 1.0
    last: Optional[BaseException] = None
    for attempt in range(3):
        try:
            request = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(request, timeout=timeout) as response:
                body = response.read(_MAX_BYTES + 1)
        except urllib.error.HTTPError as exc:
            exc.close()
            last = exc
            if exc.code not in {429, 500, 502, 503, 504}:
                raise PinError(
                    f"refusing image: registry returned HTTP {exc.code} for {what}"
                ) from exc
        except OSError as exc:
            last = exc
        else:
            if len(body) > _MAX_BYTES:
                raise PinError(f"refusing image: registry response for {what} is too large")
            return body
        if attempt < 2:
            time.sleep(delay)
            delay *= 2
    raise PinError(f"refusing image: registry request failed for {what}: {last}") from last


def bearer_token(repo: str, timeout: float = 60.0) -> str:
    check_repo(repo)
    query = urllib.parse.urlencode(
        {"service": "registry.docker.io", "scope": f"repository:{repo}:pull"}
    )
    body = _request_bytes(
        f"https://auth.docker.io/token?{query}",
        {
            "Accept": "application/json",
            "Accept-Encoding": "identity",
            "User-Agent": _USER_AGENT,
        },
        timeout,
        "token",
    )
    try:
        payload = json.loads(body)
    except json.JSONDecodeError as exc:
        raise PinError("refusing image: registry token response is not json") from exc
    token = payload.get("token") if isinstance(payload, dict) else None
    if not isinstance(token, str) or not token:
        raise PinError("refusing image: registry token response has no token")
    return token


def fetch_manifest(repo: str, digest: str, accept: str, timeout: float = 60.0) -> bytes:
    """GET one manifest and return the raw bytes. The caller checks the hash."""
    check_repo(repo)
    check_digest(digest, "manifest digest")
    token = bearer_token(repo, timeout)
    return _request_bytes(
        f"https://registry-1.docker.io/v2/{repo}/manifests/{digest}",
        {
            "Authorization": f"Bearer {token}",
            "Accept": accept,
            "Accept-Encoding": "identity",
            "User-Agent": _USER_AGENT,
        },
        timeout,
        digest,
    )


def pins_from_env(environ: Mapping[str, str]) -> tuple[str, str, str, str]:
    required = (
        "PX4_IMAGE_REPO",
        "PX4_IMAGE_INDEX_DIGEST",
        "PX4_IMAGE_AMD64_DIGEST",
        "PX4_IMAGE_ID",
    )
    missing = [name for name in required if not environ.get(name)]
    if missing:
        raise PinError("refusing image: missing " + ", ".join(missing))
    return (
        check_repo(environ["PX4_IMAGE_REPO"]),
        check_digest(environ["PX4_IMAGE_INDEX_DIGEST"], "index digest"),
        check_digest(environ["PX4_IMAGE_AMD64_DIGEST"], "amd64 digest"),
        check_digest(environ["PX4_IMAGE_ID"], "image id"),
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = sys.argv[1:] if argv is None else list(argv)
    if args:
        print("refusing image: px4_image_pin.py takes no arguments", file=sys.stderr)
        return 1
    try:
        repo, index_digest, amd64_digest, image_id = pins_from_env(os.environ)
        index_body = fetch_manifest(repo, index_digest, INDEX_ACCEPT)
        amd64_body = fetch_manifest(repo, amd64_digest, MANIFEST_ACCEPT)
        verified_image_id(index_body, index_digest, amd64_body, amd64_digest, image_id)
    except PinError as exc:
        print(exc, file=sys.stderr)
        return 1
    print(
        f"pinned {index_digest} and {amd64_digest} match image id {image_id}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
