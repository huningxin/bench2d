CC = ${LLVM}/clang
LD = ${LLVM}/llvm-link
OPT = ${LLVM}/opt
AS = ${LLVM}/llc
CFLAGS = -O0 -IBox2D_v2.2.1 -DNDEBUG=1
LFLAGS = -lstdc++ -lm

OBJECTS = \
bench2d_main.bc \
Bench2d.bc \
Box2D_v2.2.1/Box2D/Collision/b2BroadPhase.bc \
Box2D_v2.2.1/Box2D/Collision/b2CollideCircle.bc \
Box2D_v2.2.1/Box2D/Collision/b2CollideEdge.bc \
Box2D_v2.2.1/Box2D/Collision/b2CollidePolygon.bc \
Box2D_v2.2.1/Box2D/Collision/b2Collision.bc \
Box2D_v2.2.1/Box2D/Collision/b2Distance.bc \
Box2D_v2.2.1/Box2D/Collision/b2DynamicTree.bc \
Box2D_v2.2.1/Box2D/Collision/b2TimeOfImpact.bc \
Box2D_v2.2.1/Box2D/Collision/Shapes/b2ChainShape.bc \
Box2D_v2.2.1/Box2D/Collision/Shapes/b2CircleShape.bc \
Box2D_v2.2.1/Box2D/Collision/Shapes/b2EdgeShape.bc \
Box2D_v2.2.1/Box2D/Collision/Shapes/b2PolygonShape.bc \
Box2D_v2.2.1/Box2D/Common/b2BlockAllocator.bc \
Box2D_v2.2.1/Box2D/Common/b2Draw.bc \
Box2D_v2.2.1/Box2D/Common/b2Math.bc \
Box2D_v2.2.1/Box2D/Common/b2Settings.bc \
Box2D_v2.2.1/Box2D/Common/b2StackAllocator.bc \
Box2D_v2.2.1/Box2D/Common/b2Timer.bc \
Box2D_v2.2.1/Box2D/Dynamics/b2Body.bc \
Box2D_v2.2.1/Box2D/Dynamics/b2ContactManager.bc \
Box2D_v2.2.1/Box2D/Dynamics/b2Fixture.bc \
Box2D_v2.2.1/Box2D/Dynamics/b2Island.bc \
Box2D_v2.2.1/Box2D/Dynamics/b2World.bc \
Box2D_v2.2.1/Box2D/Dynamics/b2WorldCallbacks.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2CircleContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2Contact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2ContactSolver.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Contacts/b2PolygonContact.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2DistanceJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2FrictionJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2GearJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2Joint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2MouseJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2PrismaticJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2PulleyJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2RevoluteJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2RopeJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2WeldJoint.bc \
Box2D_v2.2.1/Box2D/Dynamics/Joints/b2WheelJoint.bc \
Box2D_v2.2.1/Box2D/Rope/b2Rope.bc

all: bench2d_simd bench2d_no_simd

%.bc: %.cpp
	$(CC) $(CFLAGS) -emit-llvm -o $@ -c $<

bench2d.bc: $(OBJECTS)
	$(LD) -o $@ $(OBJECTS)

bench2d_no_simd.bc: bench2d.bc
	$(OPT) -O2 -disable-loop-vectorization -disable-slp-vectorization $< -o $@

bench2d_no_simd.s: bench2d_no_simd.bc
	$(AS) $< -o $@ 

bench2d_no_simd: bench2d_no_simd.s
	$(CC) $(LFLAGS) $< -o $@

bench2d_simd.bc: bench2d.bc
	$(OPT) -O2 $< -o $@

bench2d_simd.s: bench2d_simd.bc
	$(AS) $< -o $@

bench2d_simd: bench2d_simd.s
	$(CC) $(LFLAGS) $< -o $@

clean:
	rm $(OBJECTS) bench2d.bc bench2d_no_simd.bc bench2d_simd.bc bench2d_no_simd.s bench2d_simd.s bench2d_simd bench2d_no_simd
