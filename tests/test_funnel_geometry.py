"""Physical mouth vs. base silhouette, tilted RGB-D and missing black-surface depth."""
import math
import unittest
import cv2
import numpy as np

from evoarm_rsi.funnel_geometry import detect_funnel,HEIGHT_MM


def workspace(*,tilt=0.,height=HEIGHT_MM,depth_rim=True):
    normal=np.array([0.,0.,1.]);plane={'normal_up':normal,'d_mm':115.}
    angle=math.pi-math.radians(tilt)
    R=np.array([[1.,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]])
    center=np.array([300.,-90.,height-115.]);t=center-R@np.array([0.,0.,450.])
    K=np.array([[600.,0,640.],[0,600.,360.],[0,0,1.]])
    geom={'K':K,'inverse_K':np.linalg.inv(K),'R':R,'t':t,'plane':plane,
          'normal_camera':R.T@normal,'d_camera':float(normal@t+115.)}
    theta=np.linspace(0,2*math.pi,180,endpoint=False)
    points=center+38*np.c_[np.cos(theta),np.sin(theta),np.zeros(len(theta))]
    cam=(points-t)@R;uv=cam@K.T;uv=uv[:,:2]/uv[:,2,None]
    image=np.full((720,1280,3),220,np.uint8)
    # A tab/base protruding below-right biases the old whole-blob centroid.
    cv2.fillPoly(image,[np.array([[610,380],[675,375],[715,450],[645,470]],np.int32)],(18,18,18))
    cv2.fillPoly(image,[np.round(uv).astype(np.int32)],(18,18,18))
    ys,xs=np.indices(image.shape[:2]);rays=np.c_[xs.ravel(),ys.ravel(),np.ones(xs.size)]@geom['inverse_K'].T
    denominator=rays@geom['normal_camera']
    depth=(-geom['d_camera']/denominator).reshape(image.shape[:2]).astype(np.float32)
    rim=np.zeros(depth.shape,np.uint8)
    cv2.polylines(rim,[np.round(uv).astype(np.int32)],True,255,3)
    if depth_rim:
        rim_depth=((height-geom['d_camera'])/denominator).reshape(depth.shape)
        depth[rim>0]=rim_depth[rim>0]
    else:depth[rim>0]=0
    return image,depth,geom,center


class FunnelGeometry(unittest.TestCase):
    def test_base_tab_does_not_pull_the_mouth_center(self):
        image,depth,geom,center=workspace()
        contours=cv2.findContours(cv2.inRange(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),0,85),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        moment=cv2.moments(max(contours,key=cv2.contourArea))
        self.assertGreater(math.dist([moment['m10']/moment['m00'],moment['m01']/moment['m00']],[640,360]),10)
        found=detect_funnel(image,geom,depth)
        np.testing.assert_allclose(found['mouth_center_robot_mm'],center,atol=2.)
        self.assertLess(math.dist(found['pixel_center'],[640,360]),2.)
        self.assertEqual(len(found['mouth_boundary_px']),64)
        self.assertTrue(found['depth_evidence']['depth_used'])

    def test_tilted_rim_uses_depth_to_refine_actual_height(self):
        image,depth,geom,center=workspace(tilt=30.,height=HEIGHT_MM+3)
        found=detect_funnel(image,geom,depth)
        np.testing.assert_allclose(found['mouth_center_robot_mm'],center,atol=2.)
        self.assertAlmostEqual(found['height_mm'],HEIGHT_MM+3,delta=.5)
        self.assertTrue(found['depth_evidence']['depth_used'])
        self.assertFalse(found['depth_evidence']['hole_depth_used_as_rim'])

    def test_table_seen_through_hole_does_not_become_mouth_depth(self):
        image,depth,geom,center=workspace(depth_rim=False)
        found=detect_funnel(image,geom,depth)
        self.assertFalse(found['depth_evidence']['depth_used'])
        self.assertAlmostEqual(found['height_mm'],HEIGHT_MM)
        np.testing.assert_allclose(found['mouth_center_robot_mm'],center,atol=2.)

    def test_small_neck_and_empty_scene_cannot_replace_the_large_mouth(self):
        image,depth,geom,_=workspace()
        for radius in (0,14):
            image[:]=220
            if radius:cv2.circle(image,(640,360),radius,(18,18,18),-1)
            with self.assertRaises(ValueError):detect_funnel(image,geom,depth)
