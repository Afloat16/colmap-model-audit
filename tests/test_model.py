import copy
from pathlib import Path
import tempfile
import unittest
from colmap_model_audit.core import audit
from colmap_model_audit.model import read_model


EXAMPLE=Path(__file__).resolve().parents[1]/'examples/model'


class ModelTests(unittest.TestCase):
    def setUp(self): self.model=read_model(EXAMPLE)

    def test_sparse_ids_names_with_spaces_and_geometry(self):
        self.assertEqual(self.model['images'][10]['name'],'image one.png')
        report=audit(self.model)
        self.assertEqual(report['error_count'],0)
        self.assertEqual(report['median_pinhole_residual_px'],0)
        self.assertEqual(report['view_components'],[[10,30]])

    def test_reciprocal_checks_both_directions(self):
        self.model['points'][42]['track']=[(10,0)]
        self.assertIn('missing_reciprocal_track',audit(self.model)['finding_counts'])
        self.model['points'][42]['track']=[(10,4),(999,0)]
        report=audit(self.model)
        self.assertIn('track_index_out_of_range',report['finding_counts'])
        self.assertIn('missing_track_image',report['finding_counts'])

    def test_missing_camera_and_point(self):
        self.model['images'][10]['camera_id']=999
        self.model['images'][30]['observations']=[(300,240,999)]
        report=audit(self.model)
        self.assertIn('missing_camera',report['finding_counts'])
        self.assertIn('missing_point',report['finding_counts'])

    def test_zero_quaternion_does_not_divide(self):
        self.model['images'][10]['q']=[0,0,0,0]
        self.assertIn('quaternion_norm',audit(self.model)['finding_counts'])

    def test_unknown_camera_never_silently_certified(self):
        self.model['cameras'][7]['model']='FUTURE_MODEL'
        self.assertIn('unvalidated_camera_model',audit(self.model)['finding_counts'])

    def test_merge_overlap_requires_exact_names(self):
        other=copy.deepcopy(self.model)
        other['images'][10]['name']='prefix_image one.png'
        report=audit(self.model,other=other,min_overlap=2)
        self.assertEqual(report['merge_overlap']['count'],1)
        self.assertFalse(report['merge_overlap']['meets_name_threshold'])

    def test_projection_detects_bad_pose(self):
        self.model['images'][10]['t']=[0,0,-10]
        self.assertIn('point_behind_camera',audit(self.model)['finding_counts'])

    def test_native_text_empty_observation_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'cameras.txt').write_text('7 PINHOLE 640 480 100 100 320 240\n')
            (root/'images.txt').write_text('10 1 0 0 0 0 0 0 7 a.png\n\n30 1 0 0 0 0 0 0 7 b.png\n\n')
            (root/'points3D.txt').write_text('')
            self.assertEqual(len(read_model(root)['images']),2)
            (root/'images.txt').write_text('10 1 0 0 0 0 0 0 7 a.png\n')
            with self.assertRaises(ValueError): read_model(root)

    def test_duplicate_ids_fail_instead_of_overwriting(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'cameras.txt').write_text('7 PINHOLE 640 480 100 100 320 240\n7 PINHOLE 640 480 100 100 320 240\n')
            with self.assertRaises(ValueError): read_model(root)

    def test_image_path_escape(self):
        self.model['images'][10]['name']='../escape.png'
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIn('unsafe_image_name',audit(self.model,images_root=tmp)['finding_counts'])
