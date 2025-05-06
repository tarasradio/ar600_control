class RobotDesc:
  # Legs parts params

  hip_fork_length = 0.102

  hip_length = 0.28
  shin_length = 0.28

  foot_height = 0.1055

  # Distance between hips joints
  pelvis_width = 0.176

  def CoM_height():
    return \
      (RobotDesc.hip_fork_length + \
      RobotDesc.hip_length + \
      RobotDesc.shin_length + \
      RobotDesc.foot_height)

  def pelvis_to_hip():
    return RobotDesc.pelvis_width / 2