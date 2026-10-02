import unittest
import numpy as np
from cardboard.geometry import graded_shell_grid


class GradedMeshTest(unittest.TestCase):
    def test_closed_oriented_surface_and_analytic_volume(self):
        size = np.array([.276, .204, .200])
        p, f, _ = graded_shell_grid(size)
        edges = np.concatenate([f[:,[0,1]], f[:,[1,2]], f[:,[2,0]]])
        unique, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
        self.assertTrue((counts == 2).all())
        self.assertEqual(len(p)-len(unique)+len(f), 2)
        self.assertEqual(len(np.unique(p, axis=0)), len(p))
        tri = p[f].astype(float)
        normal = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
        self.assertTrue((np.einsum('ij,ij->i', normal, tri.mean(1)) > 0).all())
        volume = np.einsum('ij,ij->i', tri[:,0], np.cross(tri[:,1],tri[:,2])).sum()/6
        self.assertAlmostEqual(volume, np.prod(size), places=8)
        area = np.linalg.norm(normal, axis=1).sum()*.5
        self.assertAlmostEqual(area, 2*(size[0]*size[1]+size[1]*size[2]+size[2]*size[0]), places=7)

    def test_counts_follow_panel_dimensions_and_grip_refinement(self):
        p, f, _ = graded_shell_grid([.276,.204,.200])
        self.assertEqual((len(p),len(f)), (1402,2800))
        for axis in (0,1):
            x = np.unique(p[:,axis]);step = np.diff(x)
            self.assertLess(step[len(step)//2], step[0])
        z = np.unique(p[:,2]);self.assertLess(np.diff(z)[-3], np.diff(z)[0])
        tri = p[f];e = np.linalg.norm(np.roll(tri,-1,axis=1)-tri,axis=2)
        cosine = (e[:,0]**2+e[:,1]**2-e[:,2]**2)/(2*e[:,0]*e[:,1])
        self.assertTrue(np.isfinite(cosine).all())
        self.assertLess((e.max(1)/e.min(1)).max(), 5.)

    def test_invalid_sizes_and_counts_rejected(self):
        for size in ([0,1,1],[1,np.nan,1]):
            with self.assertRaises(ValueError):graded_shell_grid(size)
        for counts in ((0,14,14),(18.5,14,14)):
            with self.assertRaises(ValueError):graded_shell_grid([1,1,1],counts)


if __name__ == '__main__':unittest.main()
