#!/usr/bin/env python3
"""Mathematical regression cases; run with the evaluation virtual environment."""

from pathlib import Path
import tempfile
import unittest
import warnings

import numpy as np

from trajectory_eval import align_positions, associate_timestamps, load_trajectory


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.positions = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.],
                                   [0., 0., 1.], [1., 2., 3.]])

    def test_recovers_known_rigid_transform(self):
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        translation = np.array([2., -3., 4.])
        truth = self.positions @ rotation.T + translation
        aligned, recovered_rotation, recovered_translation = align_positions(self.positions, truth)
        np.testing.assert_allclose(aligned, truth, atol=1e-12)
        np.testing.assert_allclose(recovered_rotation, rotation, atol=1e-12)
        np.testing.assert_allclose(recovered_translation, translation, atol=1e-12)

    def test_does_not_hide_scale_error(self):
        aligned, rotation, _ = align_positions(self.positions, self.positions * 2.)
        self.assertGreater(np.linalg.norm(aligned - self.positions * 2.), 1.)
        self.assertAlmostEqual(np.linalg.det(rotation), 1.)

    def test_rejects_reflection_as_rotation(self):
        reflected = self.positions * [-1., 1., 1.]
        aligned, rotation, _ = align_positions(self.positions, reflected)
        self.assertAlmostEqual(np.linalg.det(rotation), 1.)
        self.assertGreater(np.linalg.norm(aligned - reflected), 0.1)

    def test_rejects_degenerate_positions(self):
        with self.assertRaises(ValueError):
            align_positions(np.zeros((3, 3)), np.zeros((3, 3)))
        line = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
        with self.assertRaises(ValueError):
            align_positions(line, line)

    def test_association_boundary_and_truth_reuse(self):
        matches = associate_timestamps(np.array([0., .125, .25]), np.array([0., 1.]), .25)
        np.testing.assert_array_equal(matches, [[0, 0], [1, 0], [2, 0]])
        self.assertEqual(associate_timestamps(np.array([5.]), np.array([0.]), 0.).shape, (0, 2))
        for limit in [-1., np.inf, np.nan]:
            with self.assertRaises(ValueError):
                associate_timestamps(np.array([0.]), np.array([0.]), limit)

    def test_rejects_invalid_trajectory_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trajectory.txt'
            records = np.column_stack([np.arange(5), self.positions,
                                       np.zeros((5, 3)), np.ones(5)])
            np.savetxt(path, records)
            self.assertEqual(load_trajectory(path).shape, (5, 8))
            invalid = [records[:, :7], records.copy(), records.copy()]
            invalid[1][1, 0] = invalid[1][0, 0]
            invalid[2][0, 1] = np.nan
            for data in invalid:
                np.savetxt(path, data)
                with self.assertRaises(ValueError):
                    load_trajectory(path)
            path.write_text('# no samples\n')
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', UserWarning)
                with self.assertRaises(ValueError):
                    load_trajectory(path)


if __name__ == '__main__':
    unittest.main()
