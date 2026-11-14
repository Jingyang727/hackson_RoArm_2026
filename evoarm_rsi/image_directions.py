"""Camera-image semantics shared by visual reviewers, agents and controllers."""
OPPOSITE={'image_left':'image_right','image_right':'image_left','image_up':'image_down','image_down':'image_up',
          'too_high':'lower','too_low':'higher'}
PIXEL_VECTOR={'image_left':[-1,0],'image_right':[1,0],'image_up':[0,-1],'image_down':[0,1]}
CONTRACT={'frame':'unmirrored_rgb','u_positive':'image_right','v_positive':'image_down',
          'view_transform_owner':'opencode_primary_agent_with_calibrated_tools','execution_frame':'robot_firmware_mm',
          'observed_side_to_correction':{k:OPPOSITE[k] for k in PIXEL_VECTOR},
          'note':'Grok只按画面观察；主Agent负责相机视角到机械臂视角的转换，并用标定工具落实XYZ。原图纠偏效果必须符合视觉建议，已转换的XYZ原值用于执行。'}
