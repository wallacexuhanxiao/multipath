import numpy as np
from scipy.spatial.transform import Rotation

from scripts.build_taco_xhand_reference import convert_poses,rigid_alignment


def test_mano_alignment_transfers_object_pose_without_scale_or_reflection():
    rng=np.random.default_rng(3)
    source=rng.normal(size=(8,21,3))*.1
    rotation=Rotation.from_euler('xyz',[.1,-.2,.3]).as_matrix()
    offset=np.array([-.7,.04,-.05])
    target=source@rotation+offset
    found,translation,error=rigid_alignment(source,target)
    np.testing.assert_allclose(found,rotation,atol=1e-12)
    np.testing.assert_allclose(translation,offset,atol=1e-12)
    assert error.max()<1e-12 and np.linalg.det(found)>0
    pose=np.tile(np.eye(4),(8,1,1));pose[:,:3,3]=source[:,0]
    moved=convert_poses(pose,found,translation)
    np.testing.assert_allclose(moved[:,:3,3],target[:,0],atol=1e-12)
    np.testing.assert_allclose(moved[0,:3,:3],found.T,atol=1e-12)
