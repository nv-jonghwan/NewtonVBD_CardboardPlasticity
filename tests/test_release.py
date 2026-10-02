import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from cardboard.release import local_path, load_profile, solver_arguments, verify_inputs, LOCK_FILE


class ReleaseTest(unittest.TestCase):
    def test_profile_keeps_selected_solver_budget_and_resistance(self):
        p = load_profile()
        args = solver_arguments(p)
        values = {flag: args[i + 1] for i, flag in enumerate(args[:-1]) if flag.startswith('--') and not args[i + 1].startswith('--')}
        self.assertEqual(values['--iterations'], '24')
        self.assertEqual(values['--crease-friction-curvature'], '45.0')
        self.assertEqual(values['--small-bend-memory-curvature'], '0.0')
        self.assertEqual(values['--rom-basis'], 'assets/models/board4_rank8.npz')
        self.assertIn('--rom-full-after-yield', args)
        self.assertIn('--rom-fast', args)
        self.assertNotIn('outputs/', json.dumps(p))

    def test_lock_detects_changed_and_missing_input_after_relocation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'config').mkdir()
            data = root / 'input.bin'
            data.write_bytes(b'reference')
            (root / LOCK_FILE).write_text(json.dumps({'files': [{'path': 'input.bin', 'sha256': hashlib.sha256(b'reference').hexdigest()}]}))
            verify_inputs(root)
            data.write_bytes(b'corruption')
            with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
                verify_inputs(root)
            data.unlink()
            with self.assertRaisesRegex(ValueError, 'missing'):
                verify_inputs(root)

    def test_profile_paths_cannot_escape_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('/tmp/external', '../external', 'assets/../../external'):
                with self.assertRaises(ValueError):
                    local_path(name, root)
            (root / 'escape').symlink_to('/tmp')
            with self.assertRaises(ValueError):
                local_path('escape/external', root)
            self.assertEqual(local_path('assets/model.npz', root), root / 'assets/model.npz')
