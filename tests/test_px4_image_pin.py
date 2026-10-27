"""The PX4 dev image pin still holds after docker load clears RepoDigests."""

from __future__ import annotations

import json
import os
import re
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import px4_image_pin as pin  # noqa: E402

WORKFLOW = ROOT / ".github" / "workflows" / "sitl-px4.yml"
FIXTURES = ROOT / "tests" / "fixtures"
INDEX_DIGEST = "sha256:288dc1d7f5290855e86dbaf8b8f55cd17f46f27e9df2affb2056df6363bc6fb0"
AMD64_DIGEST = "sha256:82e3ddcf4bf3fa4dbc0be6489c6de4158d0efbc8530182c691184302027335ce"
IMAGE_ID = "sha256:4592d1b50a5a094e780e573eeaaa932f35dc981db47bce1b65dbccae2a32fb42"


def _workflow_value(name: str, text: str) -> str:
    match = re.search(rf"(?m)^[ \t]*{re.escape(name)}: (.+)$", text)
    if match is None:
        raise AssertionError(name)
    return match.group(1).strip()


def _canonical(payload: object) -> tuple[bytes, str]:
    body = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode()
    return body, pin.sha256_digest(body)


def _manifest(config_digest: str, layer_digest: str) -> dict[str, object]:
    return {
        "schemaVersion": 2,
        "mediaType": "application/vnd.docker.distribution.manifest.v2+json",
        "config": {
            "mediaType": "application/vnd.docker.container.image.v1+json",
            "digest": config_digest,
            "size": 1,
        },
        "layers": [
            {
                "mediaType": "application/vnd.docker.image.rootfs.diff.tar.gzip",
                "digest": layer_digest,
                "size": 1,
            }
        ],
    }


class ManifestPinTests(unittest.TestCase):
    def setUp(self) -> None:
        self.index_body = (FIXTURES / "px4-dev-v1.17.0-index.json").read_bytes()
        self.amd64_body = (FIXTURES / "px4-dev-v1.17.0-amd64.json").read_bytes()

    def test_checked_in_manifests_match_the_workflow_pins(self) -> None:
        text = WORKFLOW.read_text(encoding="utf-8")
        self.assertEqual(_workflow_value("PX4_IMAGE_INDEX_DIGEST", text), INDEX_DIGEST)
        self.assertEqual(_workflow_value("PX4_IMAGE_AMD64_DIGEST", text), AMD64_DIGEST)
        self.assertEqual(_workflow_value("PX4_IMAGE_ID", text), IMAGE_ID)
        self.assertEqual(pin.sha256_digest(self.index_body), INDEX_DIGEST)
        self.assertEqual(pin.sha256_digest(self.amd64_body), AMD64_DIGEST)
        self.assertEqual(
            pin.verified_image_id(
                self.index_body,
                INDEX_DIGEST,
                self.amd64_body,
                AMD64_DIGEST,
                IMAGE_ID,
            ),
            IMAGE_ID,
        )
        self.assertIn(b"\n", self.index_body)
        self.assertNotIn(b"\r", self.index_body)
        self.assertNotIn(b"\r", self.amd64_body)

    def test_a_changed_manifest_byte_is_refused(self) -> None:
        tampered = bytearray(self.amd64_body)
        tampered[0] ^= 0x01
        with self.assertRaises(pin.PinError) as caught:
            pin.verified_image_id(
                self.index_body,
                INDEX_DIGEST,
                bytes(tampered),
                AMD64_DIGEST,
                IMAGE_ID,
            )
        self.assertIn("hashed to", str(caught.exception))

    def test_a_different_image_id_is_refused(self) -> None:
        other = "sha256:" + "ab" * 32
        with self.assertRaises(pin.PinError) as caught:
            pin.verified_image_id(
                self.index_body,
                INDEX_DIGEST,
                self.amd64_body,
                AMD64_DIGEST,
                other,
            )
        self.assertIn("config digest", str(caught.exception))

    def test_index_must_list_the_amd64_manifest(self) -> None:
        image_id = "sha256:" + "11" * 32
        layer = "sha256:" + "22" * 32
        amd64_body, amd64_digest = _canonical(_manifest(image_id, layer))
        other = "sha256:" + "33" * 32
        index_body, index_digest = _canonical(
            {
                "manifests": [
                    {
                        "digest": other,
                        "platform": {"architecture": "amd64", "os": "linux"},
                    }
                ]
            }
        )
        with self.assertRaises(pin.PinError) as caught:
            pin.verified_image_id(
                index_body, index_digest, amd64_body, amd64_digest, image_id
            )
        self.assertIn("does not list", str(caught.exception))

    def test_layer_without_a_digest_is_refused(self) -> None:
        image_id = "sha256:" + "44" * 32
        amd64_body, amd64_digest = _canonical(
            {
                "config": {"digest": image_id},
                "layers": [{"size": 1}],
            }
        )
        index_body, index_digest = _canonical(
            {
                "manifests": [
                    {
                        "digest": amd64_digest,
                        "platform": {"architecture": "amd64", "os": "linux"},
                    }
                ]
            }
        )
        with self.assertRaises(pin.PinError) as caught:
            pin.verified_image_id(
                index_body, index_digest, amd64_body, amd64_digest, image_id
            )
        self.assertIn("layer without a digest", str(caught.exception))

    def test_repo_and_digest_shape(self) -> None:
        self.assertEqual(pin.check_repo("px4io/px4-dev"), "px4io/px4-dev")
        with self.assertRaises(pin.PinError):
            pin.check_repo("px4io/px4-dev/extra")
        with self.assertRaises(pin.PinError):
            pin.check_digest("sha256:abcd", "image id")

    def test_main_accepts_fixture_bytes_without_calling_the_registry(self) -> None:
        seen: list[str] = []

        def fake(repo: str, digest: str, accept: str, timeout: float = 60.0) -> bytes:
            del timeout
            self.assertEqual(repo, "px4io/px4-dev")
            seen.append(digest)
            if digest == INDEX_DIGEST:
                self.assertEqual(accept, pin.INDEX_ACCEPT)
                return self.index_body
            if digest == AMD64_DIGEST:
                self.assertEqual(accept, pin.MANIFEST_ACCEPT)
                return self.amd64_body
            raise AssertionError(digest)

        env = {
            "PX4_IMAGE_REPO": "px4io/px4-dev",
            "PX4_IMAGE_INDEX_DIGEST": INDEX_DIGEST,
            "PX4_IMAGE_AMD64_DIGEST": AMD64_DIGEST,
            "PX4_IMAGE_ID": IMAGE_ID,
        }
        with mock.patch.dict(os.environ, env, clear=False):
            with mock.patch.object(pin, "fetch_manifest", side_effect=fake):
                self.assertEqual(pin.main([]), 0)
        self.assertEqual(seen, [INDEX_DIGEST, AMD64_DIGEST])

    def test_main_refuses_arguments_and_a_mismatched_id(self) -> None:
        self.assertEqual(pin.main(["--pull"]), 1)
        env = {
            "PX4_IMAGE_REPO": "px4io/px4-dev",
            "PX4_IMAGE_INDEX_DIGEST": INDEX_DIGEST,
            "PX4_IMAGE_AMD64_DIGEST": AMD64_DIGEST,
            "PX4_IMAGE_ID": "sha256:" + "ab" * 32,
        }

        def fake(repo: str, digest: str, accept: str, timeout: float = 60.0) -> bytes:
            del repo, accept, timeout
            if digest == INDEX_DIGEST:
                return self.index_body
            return self.amd64_body

        with mock.patch.dict(os.environ, env, clear=False):
            with mock.patch.object(pin, "fetch_manifest", side_effect=fake):
                self.assertEqual(pin.main([]), 1)


class WorkflowPinTests(unittest.TestCase):
    def test_cache_hit_keeps_the_digest_pin_and_runs_the_image_id(self) -> None:
        text = WORKFLOW.read_text(encoding="utf-8")
        self.assertIn("Refuse hardware arming", text)
        self.assertIn("scripts/px4_image_pin.py", text)
        before_fly, fly = text.split("- name: Fly the square", 1)
        step = before_fly.split("- name: Load or pull the pinned image", 1)[1]
        self.assertLess(
            step.index("python3 scripts/px4_image_pin.py"),
            step.index("docker load"),
        )
        self.assertIn(
            'docker pull "${PX4_IMAGE_REPO}@${PX4_IMAGE_AMD64_DIGEST}"',
            step,
        )
        self.assertNotIn('docker pull "$image"', step)
        self.assertNotIn('docker pull "${PX4_IMAGE_REPO}:${PX4_IMAGE_TAG}"', step)
        self.assertLess(step.index("{{.Id}}"), step.index("{{json .RepoDigests}}"))
        self.assertIn("require_repo_digest", step)
        empty = step.split('"[]"|"null")', 1)[1]
        self.assertLess(
            empty.index('"$require_repo_digest" -eq 1'),
            empty.index("refusing image: digest is not the pinned v1.17.0 image"),
        )
        self.assertIn("refusing image: id ${actual} is not the pinned v1.17.0 image", step)
        self.assertIn('"${PX4_IMAGE_ID}"', fly)
        self.assertIn("--pull never", fly)
        self.assertLess(fly.index("--pull never"), fly.index('"${PX4_IMAGE_ID}"'))
        self.assertNotIn("${PX4_IMAGE_REPO}:${PX4_IMAGE_TAG}", fly)
        self.assertIn("${OHHO_ARM_HARDWARE+x}", before_fly)
