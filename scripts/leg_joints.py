# Describes humanoid robot leg joints

def tf_leg_joints(hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll, side):
  if side == 'left': # inverce 'roll' for left leg
    hip_roll = -hip_roll
    ankle_roll = -ankle_roll
  # inverce joints angles (for urdf model)
  return (-hip_yaw, -hip_roll, -hip_pitch, -knee_pitch, -ankle_pitch, -ankle_roll)