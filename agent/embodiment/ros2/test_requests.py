"""Request builders for tests using the public group motion contract."""


def pose_move(arm_id, frame_id, position, orientation, duration=3.):
  return dict(arm_ids=[arm_id], targets=[dict(kind='pose', frame_id=frame_id,
      position=position, orientation=orientation)], duration=duration)


def named_move(arm_id, name='ready', duration=3.):
  return dict(arm_ids=[arm_id], targets=[dict(kind='named', name=name)], duration=duration)
