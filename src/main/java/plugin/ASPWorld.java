package plugin;

import ab.vision.ABObject;
import ab.vision.ABType;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;

import java.util.Collection;

public class ASPWorld {
	Collection<ABObject> objects;
	Collection<PolygonShape> hills;

	public World toWorld(int scale, boolean createAutoGround, int hiddenObjectId) {
		Vec2 gravity = new Vec2(0f, 9.8f);
		World world = new World(gravity);

		for (ABObject o : objects) {
			if (createAutoGround && o.getType().equals(ABType.Ground)) {
				continue;
			}

			if (o.id == hiddenObjectId) {
				continue;
			}

			// FIXME: Sclaing?
			//o.(scale);

			// Define the dynamic body. We set its position and call the body factory.
			BodyDef bodyDef = new BodyDef();
			bodyDef.type = BodyType.DYNAMIC;
			bodyDef.position = Util.pointToVec2(o.getCenter());
			Body body = world.createBody(bodyDef);

			// Define another box shape for our dynamic body.
			PolygonShape dynamicBox = new PolygonShape();
			dynamicBox.setAsBox(o.width / 2.0f, o.height / 2.0f);

			// Define the dynamic body fixture.
			FixtureDef fixtureDef = new FixtureDef();
			fixtureDef.shape = dynamicBox;

			// Set the box density to be non-zero, so it will be dynamic.
			fixtureDef.density = 1.0f;
			fixtureDef.restitution = 0;

			// Add the shape to the body.
			body.createFixture(fixtureDef);

			// Change the rotation
			body.setTransform(body.getPosition(), (float) o.angle);

			//The back link from body to object.
			if (!o.getType().equals(ABType.Ground)) {
				body.setUserData(o.id);
			}
		}

		if (createAutoGround) {
			// Define the ground body.
			BodyDef groundBodyDef = new BodyDef();
			groundBodyDef.position.set(0.0f, maxYPlusH(world));
			Body groundBody = world.createBody(groundBodyDef);
			groundBody.setUserData(null);
			PolygonShape groundBox = new PolygonShape();
			groundBox.setAsBox(1000.0f, 0.0f);
			groundBody.createFixture(groundBox, 1.0f);
		}

		for (PolygonShape hill : hills) {
			BodyDef hillDef = new BodyDef();
			hillDef.position.set(0.0f, 0.0f); //maxYPlusH(world));
			Body hillBody = world.createBody(hillDef);
			hillBody.setUserData(null);
			hillBody.createFixture(hill, 1.0f);
		}

		return world;
	}

	public static float maxXPlusW(World world) {
		float m = 0;

		for (Body b = world.getBodyList(); b != null; b = b.getNext()) {
			for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
				m = Math.max(f.getAABB(0).upperBound.x, m);
			}
		}

		return m;
	}

	public static float maxYPlusH(World world) {
		float m = 0;

		for (Body b = world.getBodyList(); b != null; b = b.getNext()) {
			for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
				m = Math.max(f.getAABB(0).upperBound.y, m);
			}
		}

		return m;
	}

	public static int getId(Body body) {
		Object userData = body.getUserData();

		if (userData == null) {
			throw new IllegalArgumentException("Given body has no id");
		}

		return (Integer) userData;
	}

	public ABObject getById(int id) {
		for (ABObject o : objects) {
			if (o.id == id) {
				return o;
			}
		}
		return null;
	}
}
