"""Calibrated image -> ground plane, lateral-right x and forward-from-front y."""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class GeometryEstimate:
    distance_m: object = None
    line: object = None
    error_m: float = 0.
    validated: bool = False
    status: str = 'unsupported'
    # A lower bound is not an exact distance. Include calibration uncertainty before
    # using it, and stop if it cannot support the actual-speed stopping distance.
    min_distance_m: object = None


class GroundGeometry:
    # 把已标定像素映射到车头地面坐标：x向右、y向前，单位米。
    #
    # review重点：分辨率、标定覆盖区域、有效距离与车头宽度限制；超出范围
    # 必须拒绝测距，不能外推一个看似合理的距离。
    def __init__(self,spec):
        self.spec=spec
        self.width,self.height=spec['width'],spec['height']
        self.H=np.asarray(spec['homography'],dtype=float)
        self.hull=np.asarray(spec['image_hull'],np.float32).reshape(-1,2)
        self.half_width=float(spec['front_half_width_m'])
        self.min_range,self.max_range=spec['range_m']
        self.error=float(spec['error_m']);self.fit_error=float(spec['max_fit_error_m'])
        self.extrapolation=float(spec.get('max_lateral_extrapolation_m',0.))
        self.validated=spec['validated']
        # The homography maps into the BUMPER frame; odometry pose is the BASE frame. The
        # overhang between them is the only thing joining the two, so it must be carried by
        # the calibration itself -- not supplied separately at runtime, where the two could
        # silently disagree and shift every distance by the difference.
        self.front_offset=spec.get('front_offset_m')
        if self.front_offset is None:
            raise ValueError('Ground calibration must carry the surveyed front_offset_m it '
                             'was built with')
        if not math.isfinite(float(self.front_offset)) or not 0<=float(self.front_offset)<=1.:
            raise ValueError('Ground calibration front_offset_m must be 0..1 m')
        self.front_offset=float(self.front_offset)
        self.K=np.asarray(spec.get('camera_matrix',[]),float)
        self.distortion=np.asarray(spec.get('distortion_coefficients',[]),float)
        if (self.H.shape!=(3,3) or not np.isfinite(self.H).all() or abs(np.linalg.det(self.H))<1e-12 or
                self.hull.shape[0]<4 or not np.isfinite(self.hull).all() or
                abs(cv2.contourArea(self.hull))<1. or type(self.validated) is not bool):
            raise ValueError('Invalid nondegenerate ground calibration')
        scalars=(self.half_width,self.min_range,self.max_range,self.error,self.fit_error,self.extrapolation)
        if not all(math.isfinite(x) for x in scalars) or not (self.half_width>0 and 0<self.min_range<self.max_range and
                self.error>=0 and self.fit_error>0 and self.extrapolation>=0):
            raise ValueError('Invalid ground calibration domain')
        if spec.get('pixel_space','raw') not in ('raw','rectified'):
            raise ValueError('pixel_space must be raw or rectified')
        if spec.get('pixel_space')=='rectified' and (self.K.shape!=(3,3) or not np.isfinite(self.K).all() or
                not self.distortion.size or not np.isfinite(self.distortion).all()):
            raise ValueError('Rectified calibration needs camera intrinsics and distortion')

    def rectified(self,points):
        p=np.asarray(points,float).reshape(-1,2)
        if self.spec.get('pixel_space')=='rectified':
            p=cv2.undistortPoints(p.reshape(-1,1,2),self.K,self.distortion,P=self.K).reshape(-1,2)
        return p

    def measure(self,points,width,height):
        def reject(reason):return GeometryEstimate(status=reason)
        if (width,height)!=(self.width,self.height):return reject('resolution_mismatch')
        p=self.rectified(points)
        if len(p)<2 or not np.isfinite(p).all():return reject('invalid_points')
        p=np.array([q for q in p if cv2.pointPolygonTest(self.hull,(float(q[0]),float(q[1])),False)>=0])
        if len(p)<2:return reject('outside_image_calibration')
        q=np.column_stack([p,np.ones(len(p))])@self.H.T
        if np.any(np.abs(q[:,2])<1e-8):return reject('projection_horizon')
        q=q[:,:2]/q[:,2,None]
        vx,vy,x,y=cv2.fitLine(q.astype(np.float32),cv2.DIST_HUBER,0,.001,.001).flatten()
        normal=np.array([-vy,vx],float);normal/=np.linalg.norm(normal)
        if normal[1]<0:normal=-normal
        if normal[1]<.35:return reject('line_parallel_to_travel')
        c=-float(normal@np.array([x,y]))
        residual=np.abs(q@normal+c)
        fit=float(np.quantile(residual,.90))
        if fit>self.fit_error:return reject('ground_line_fit_uncertain')
        a,b=normal
        # Only the *nearer* front corner sets the stop distance; the far corner may be
        # outside the observed span without affecting it. Requiring the whole corridor was
        # rejecting usable partially-seen (lateral/off-centre) lines, so require instead
        # that the critical corner is observed, allowing a measured bounded extrapolation.
        near_neg=-(a*(-self.half_width)+c)/b
        near_pos=-(a*self.half_width+c)/b
        critical=-self.half_width if near_neg<=near_pos else self.half_width
        if critical<float(q[:,0].min())-self.extrapolation or critical>float(q[:,0].max())+self.extrapolation:
            return reject('outside_corridor_support')
        d=min(near_neg,near_pos)
        error=(self.error+fit)/float(b)
        if not self.min_range<=d<=self.max_range:
            if d>self.max_range and self.max_range>error:
                return GeometryEstimate(min_distance_m=float(self.max_range-error),
                                        error_m=error,validated=self.validated,status='beyond_ground_range')
            return reject('outside_ground_range')
        # A slanted line turns a longitudinal range error into a larger normal error.
        # Dividing by the normal's forward component is the projection; it is always the
        # conservative direction (b<1), so a shallow viewing angle cannot fake precision.
        return GeometryEstimate(float(d),(float(a),float(b),c),
                                error,self.validated,'ground')
