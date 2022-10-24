import pymunk
import pymunk.constraints
size = w, h = 1000, 430
class PinJoint:
    def __init__(self, space, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.PinJoint(b, b2, a, a2)
        space.add(joint)


class PivotJoint:
    def __init__(self, space, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        joint = pymunk.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)


class SlideJoint:
    def __init__(self, space, b, b2, a=(0, 0), a2=(0, 0), min=50, max=100, collide=True):
        joint = pymunk.SlideJoint(b, b2, a, a2, min, max)
        joint.collide_bodies = collide
        space.add(joint)


class GrooveJoint:
    def __init__(self, space, a, b, groove_a, groove_b, anchor_b):
        joint = pymunk.constraints.GrooveJoint(
            a, b, groove_a, groove_b, anchor_b)
        joint.collide_bodies = False
        space.add(joint)


class DampedRotarySpring:
    def __init__(self, space, b, b2, angle, stiffness, damping):
        joint = pymunk.DampedRotarySpring(
            b, b2, angle, stiffness, damping)
        space.add(joint)


class RotaryLimitJoint:
    def __init__(self, space, b, b2, min, max, collide=True):
        joint = pymunk.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)


class RatchetJoint:
    def __init__(self, space, b, b2, phase, ratchet):
        joint = pymunk.GearJoint(b, b2, phase, ratchet)
        space.add(joint)


class SimpleMotor:
    def __init__(self, space, b, b2, rate):
        joint = pymunk.SimpleMotor(b, b2, rate)
        space.add(joint)


class GearJoint:
    def __init__(self, space, b, b2, phase, ratio):
        joint = pymunk.GearJoint(b, b2, phase, ratio)
        space.add(joint)


class Segment:
    def __init__(self, space, p0, v, radius=5, color = (0, 255, 0, 0), group = 1):
        # moment = pymunk.moment_for_segment(1,(0,0),v,radius)
        self.body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.body.position = p0
        self.shape = pymunk.Segment(self.body, (0, 0), v, radius)
        self.shape.density = 1
        self.shape.elasticity = 1
        self.shape.friction = 1
        self.shape.filter = pymunk.ShapeFilter(group)
        self.shape.color = color
        space.add(self.body, self.shape)


class Circle:
    def __init__(self, space, pos, radius=10, group = 1, color = (0, 0, 0, 0)):
        self.body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.body.position = pos
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.density = 1
        self.shape.friction = 0
        self.shape.elasticity = 1
        self.shape.color = color
        self.shape.filter = pymunk.ShapeFilter(group)
        space.add(self.body, self.shape)

class Dot:
    def __init__(self, space, pos, group = 4, color = (0, 0, 0, 0)):
        self.body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.body.position = pos
        self.shape = pymunk.Circle(self.body, 2)
        self.shape.collide_bodies = False
        self.shape.filter = pymunk.ShapeFilter(group)
        self.shape.color = color
        space.add(self.body, self.shape)






class Box:
    def __init__(self, space, p0=(0, 0), p1=(w, h), d=4):
        x0, y0 = p0
        x1, y1 = p1
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i in range(4):
            segment = pymunk.Segment(
                space.static_body, pts[i], pts[(i+1) % 4], d)
            segment.elasticity = 1
            segment.friction = 1
            space.add(segment)




class Poly:
    def __init__(self, space, pos, vertices, color = (255, 0, 0, 0) ,group = 1):
        self.body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.body.position = pos

        self.shape = pymunk.Poly(self.body, vertices)
        self.shape.filter = pymunk.ShapeFilter(group=1)
        self.shape.density = 1
        self.shape.elasticity = 1
        self.shape.color = color
        self.shape.filter = pymunk.ShapeFilter(group)
        space.add(self.body, self.shape)


class Rectangle:
    def __init__(self, space, pos, size=(80, 50)):
        self.body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.body.position = pos

        shape = pymunk.Poly.create_box(self.body, size)
        shape.density = 1
        shape.elasticity = 1
        shape.friction = 1
        space.add(self.body, shape)
