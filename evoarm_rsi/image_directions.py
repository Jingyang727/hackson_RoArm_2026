"""Camera-image semantics shared by visual reviewers, agents and controllers."""
OPPOSITE={'image_left':'image_right','image_right':'image_left','image_up':'image_down','image_down':'image_up',
          'too_high':'lower','too_low':'higher'}
PIXEL_VECTOR={'image_left':[-1,0],'image_right':[1,0],'image_up':[0,-1],'image_down':[0,1]}
CONTRACT={'frame':'unmirrored_rgb','u_positive':'image_right','v_positive':'image_down',
          'observed_side_to_correction':{k:OPPOSITE[k] for k in PIXEL_VECTOR},
          'note':'偏差方向取反得到纠正方向；若输入已经是建议移动方向，不再取反。机器人XYZ由当前标定转换，禁止再按机器人视角镜像。'}
