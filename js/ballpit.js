var Vec2 = Box2D.Common.Math.b2Vec2
  , BodyDef = Box2D.Dynamics.b2BodyDef
  , Body = Box2D.Dynamics.b2Body
  , FixtureDef = Box2D.Dynamics.b2FixtureDef
  , Fixture = Box2D.Dynamics.b2Fixture
  , World = Box2D.Dynamics.b2World
  , MassData = Box2D.Collision.Shapes.b2MassData
  , PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  , CircleShape = Box2D.Collision.Shapes.b2CircleShape
  , RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
  , Settings = Box2D.Common.b2Settings
  , b2Math = Box2D.Common.Math.b2Math;
  ;

var world;

var PYRAMID_SIZE = 40;

function init() {
  var gravity = new Vec2(0, -10);
  world = new World(gravity, true);

  {
    var bd = new BodyDef();
    var ground = world.CreateBody(bd);
  }

  {
    var bd = new BodyDef();
    bd.type = Body.b2_dynamicBody;
    bd.allowSleep = false;
    bd.position.Set(0.0, 10.0);
    var body = world.CreateBody(bd);

    var shape = new PolygonShape();
    shape.SetAsOrientedBox(0.5, 10.0, new Vec2(10.0, 0.0), 0.0);
    body.CreateFixture2(shape, 5.0);

    shape.SetAsOrientedBox(0.5, 10.0, new Vec2(-10.0, 0.0), 0.0);
    body.CreateFixture2(shape, 5.0);

    shape.SetAsOrientedBox(10.0, 0.5, new Vec2(0.0, 10.0), 0.0);
    body.CreateFixture2(shape, 5.0);

    shape.SetAsOrientedBox(10.0, 0.5, new Vec2(0.0, -10.0), 0.0);
    body.CreateFixture2(shape, 5.0);

    var jd = new RevoluteJointDef();
    jd.bodyA = ground;
    jd.bodyB = body;
    jd.localAnchorA.Set(0.0, 10.0);
    jd.localAnchorB.Set(0.0, 0.0);
    jd.referenceAngle = 0.0;
    jd.motorSpeed = 0.10 * Settings.b2_pi;
    jd.maxMotorTorque = 1e8;
    jd.enableMotor = true;
    var joint = world.CreateJoint(jd);
  }

  // add the balls in a pyramid
  {
    var step   = (20.0 - 1.0)/PYRAMID_SIZE;
    var radius = step/2 - 0.10;
    var shape = new CircleShape();
    shape.m_radius = radius;

    var bottomLeft = new Vec2(-9.5 + radius, 0.5 + radius);
    for (var y = 0; y < PYRAMID_SIZE; ++y) {
      for (var x = y; x < PYRAMID_SIZE; ++x) {
        var offset = new Vec2(x*step - y*step/2, y*step);
        var bd = new BodyDef();
        bd.type = Body.b2_dynamicBody;
        bd.position = b2Math.AddVV(bottomLeft, offset);
        var body = world.CreateBody(bd);
        body.CreateFixture2(shape, 5.0);
      }
    }
  }
}

function step() {
  world.Step(1 / 60, 3, 3);
}

