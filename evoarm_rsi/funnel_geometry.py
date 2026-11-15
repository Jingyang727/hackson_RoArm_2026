"""Fit the large circular rim, independently of the dark base/neck silhouette.

RGB contours are rectified to the calibrated rim plane before circle fitting.
Aligned depth may refine center and height only with distributed rim support;
depth looking through the hole is never interpreted as mouth height.
"""
from __future__ import annotations

import math
import cv2
import numpy as np

RADIUS_MM = 38.
HEIGHT_MM = 45.4


def _basis(geometry):
    normal=np.asarray(geometry['plane']['normal_up'],float)
    u=np.array([1.,0.,0.]);u-=normal*np.dot(u,normal);u/=np.linalg.norm(u)
    return normal,u,np.cross(normal,u)


def _project(points,geometry):
    camera=(np.asarray(points)-geometry['t'])@geometry['R']
    pixels=camera@geometry['K'].T
    return pixels[:,:2]/pixels[:,2,None]


def _on_plane(pixels,height,geometry):
    rays=np.c_[pixels,np.ones(len(pixels))]@geometry['inverse_K'].T
    distance=(height-geometry['d_camera'])/(rays@geometry['normal_camera'])
    return (rays*distance[:,None])@geometry['R'].T+geometry['t']


def _circle(points):
    """Deterministic robust fit; require evidence around at least half the rim."""
    if len(points)<18:return None
    points=points[::max(1,len(points)//450)]
    rng=np.random.default_rng(17)
    triples=points[rng.integers(0,len(points),(300,3))]
    a=triples[:,1]-triples[:,0];b=triples[:,2]-triples[:,0]
    det=2*(a[:,0]*b[:,1]-a[:,1]*b[:,0])
    valid=np.abs(det)>2
    a,b,base,det=a[valid],b[valid],triples[valid,0],det[valid]
    aa=np.sum(a*a,axis=1);bb=np.sum(b*b,axis=1)
    centers=base+np.c_[(aa*b[:,1]-bb*a[:,1])/det,(a[:,0]*bb-b[:,0]*aa)/det]
    radii=np.linalg.norm(centers-base,axis=1)
    valid=(radii>=RADIUS_MM-8)&(radii<=RADIUS_MM+8)
    centers,radii=centers[valid],radii[valid]
    if not len(centers):return None
    distances=np.linalg.norm(points[None,:,:]-centers[:,None,:],axis=2)
    masks=np.abs(distances-radii[:,None])<2.5
    scores=masks.sum(axis=1)*np.exp(-np.abs(radii-RADIUS_MM)/12)
    for index in np.argsort(scores)[-8:][::-1]:
        center=centers[index].copy();radius=radii[index]
        for _ in range(6):
            delta=points-center;distance=np.linalg.norm(delta,axis=1)
            inliers=(np.abs(distance-radius)<3)&(distance>1e-6)
            if inliers.sum()<18:break
            J=np.c_[-delta[inliers]/distance[inliers,None],-np.ones(inliers.sum())]
            step=np.linalg.lstsq(J,distance[inliers]-radius,rcond=None)[0]
            center-=step[:2];radius-=step[2]
            if np.linalg.norm(step)<.01:break
        distance=np.linalg.norm(points-center,axis=1);inliers=np.abs(distance-radius)<3
        if inliers.sum()<18 or not RADIUS_MM-8<=radius<=RADIUS_MM+8:continue
        angles=np.arctan2(*(points[inliers]-center).T[::-1])
        bins=np.unique(np.floor((angles+math.pi)/(2*math.pi)*24).astype(int)%24)
        if len(bins)<12:continue
        residual=float(np.median(np.abs(distance[inliers]-radius)))
        return {'center':center,'radius':float(radius),'inliers':inliers,
                'coverage':len(bins)/24,'residual_mm':residual,
                'score':len(bins)/24*float(inliers.mean())*math.exp(-abs(radius-RADIUS_MM)/12)}
    return None


def _depth_refine(depth,center,radius,geometry,basis):
    normal,u,v=basis
    evidence={'method':'rgb_rim_circle_and_saved_table','depth_used':False,
              'valid_samples':0,'rim_samples':0,'hole_depth_used_as_rim':False,
              'median_rim_height_mm':None,'reason':'口沿深度缺失；使用RGB口沿和实测高度'}
    if depth is None:return center,evidence
    theta=np.linspace(0,2*math.pi,96,endpoint=False)
    ring=center+radius*(np.cos(theta)[:,None]*u+np.sin(theta)[:,None]*v)
    mask=np.zeros(depth.shape,np.uint8)
    cv2.polylines(mask,[np.round(_project(ring,geometry)).astype(np.int32)],True,255,9)
    ys,xs=np.nonzero((mask>0)&np.isfinite(depth)&(depth>0))
    evidence['valid_samples']=int(len(xs))
    if len(xs)<18:return center,evidence
    stride=max(1,len(xs)//2500);xs,ys=xs[::stride],ys[::stride]
    rays=np.c_[xs,ys,np.ones(len(xs))]@geometry['inverse_K'].T
    points=(rays*depth[ys,xs,None])@geometry['R'].T+geometry['t']
    heights=points@normal+geometry['plane']['d_mm']
    plausible=np.abs(heights-HEIGHT_MM)<=8
    points,heights=points[plausible],heights[plausible]
    evidence['rim_samples']=int(len(points))
    if len(points)<18:
        evidence['reason']='有效深度未形成口沿；排除孔内、桌面和底座深度'
        return center,evidence
    height=float(np.median(heights));evidence['median_rim_height_mm']=height
    fit=_circle(np.c_[points@u,points@v])
    if fit is None:
        evidence['reason']='口沿深度分布不足，未采用不稳定的圆心修正'
        return center,evidence
    estimate=u*fit['center'][0]+v*fit['center'][1]+normal*(height-geometry['plane']['d_mm'])
    spread=float(np.median(np.abs(heights-height)))
    evidence.update(fit_residual_mm=fit['residual_mm'],angular_coverage=fit['coverage'],
                    fitted_radius_mm=fit['radius'],height_spread_mm=spread)
    if np.linalg.norm(estimate-center)>15 or spread>3.5:
        evidence['reason']='深度拟合与可见口沿不一致，保留RGB圆心'
        return center,evidence
    evidence.update(method='rgbd_distributed_rim_circle_fit',depth_used=True,reason=None)
    return estimate,evidence


def detect_funnel(image,geometry,depth_mm=None):
    normal,u,v=basis=_basis(geometry)
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    dark=cv2.inRange(gray,0,85)
    dark=cv2.morphologyEx(dark,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    components=cv2.findContours(dark,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
    candidates=[]
    for contour in components:
        x,y,w,h=cv2.boundingRect(contour)
        if cv2.contourArea(contour)<100 or min(x,y)<2 or x+w>=image.shape[1]-2 or y+h>=image.shape[0]-2:continue
        if not .35<w/max(h,1)<2.8:continue
        pixels=contour[:,0,:].astype(float)
        world=_on_plane(pixels,HEIGHT_MM,geometry)
        # Reject large scene-wide components such as a connected chessboard.
        extent=np.ptp(np.c_[world@u,world@v],axis=0)
        if max(extent)>210 or min(extent)<30:continue
        outlines=[pixels]
        edges=cv2.Canny(gray[y:y+h,x:x+w],20,65)
        for edge in cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[-2]:
            if len(edge)>=24:outlines.append(edge[:,0,:]+[x,y])
        for outline in outlines:
            world=_on_plane(np.asarray(outline,float),HEIGHT_MM,geometry)
            fit=_circle(np.c_[world@u,world@v])
            if fit is None:continue
            center=u*fit['center'][0]+v*fit['center'][1]+normal*(HEIGHT_MM-geometry['plane']['d_mm'])
            pixel=_project([center],geometry)[0]
            if not (x<=pixel[0]<x+w and y<=pixel[1]<y+h):continue
            rgb_pixel=pixel.tolist()
            center,rim=_depth_refine(depth_mm,center,fit['radius'],geometry,basis)
            theta=np.linspace(0,2*math.pi,64,endpoint=False)
            boundary=center+RADIUS_MM*(np.cos(theta)[:,None]*u+np.sin(theta)[:,None]*v)
            projected=_project(boundary,geometry)
            pixel=_project([center],geometry)[0]
            score=fit['score']+(1. if rim['depth_used'] else 0.)
            candidates.append({'pixel_center':pixel.tolist(),'rgb_pixel_center':rgb_pixel,'mouth_center_robot_mm':center.tolist(),
                'mouth_boundary_px':projected.tolist(),'bbox_px':[x,y,w,h],'score':score,
                'observed_diameter_px':float(np.linalg.norm(np.ptp(projected,axis=0))/math.sqrt(2)*fit['radius']/RADIUS_MM),
                'expected_diameter_px':float(max(np.ptp(projected,axis=0))),
                'radius_mm':RADIUS_MM,'height_mm':float(center@normal+geometry['plane']['d_mm']),
                'rim_fit':{'method':'rgb_contour_rectified_circle','radius_mm':fit['radius'],
                           'residual_mm':fit['residual_mm'],'angular_coverage':fit['coverage']},
                'depth_evidence':rim})
    candidates.sort(key=lambda c:c['score'],reverse=True)
    distinct=[]
    for candidate in candidates:
        if all(math.dist(candidate['mouth_center_robot_mm'],other['mouth_center_robot_mm'])>15 for other in distinct):
            distinct.append(candidate)
    if not distinct:raise ValueError('当前没有找到完整的大漏斗口沿；请让大开口出现在画面中后重新观察。')
    selected=distinct[0];selected['candidate_count']=len(distinct)
    selected['ambiguous']=len(distinct)>1 and distinct[1]['score']>.8*selected['score']
    return selected


def draw_mouth(image,funnel,color=(100,230,100)):
    """Show the projected usable opening and its center, not a fixed-size marker."""
    boundary=funnel.get('mouth_boundary_px')
    if boundary:cv2.polylines(image,[np.round(boundary).astype(np.int32)],True,color,2)
    pixel=tuple(np.round(funnel['pixel_center']).astype(int))
    cv2.drawMarker(image,pixel,color,cv2.MARKER_CROSS,12,2)
    cv2.putText(image,'FUNNEL MOUTH',pixel,cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
    return image
