package plugin;

import ab.planner.TrajectoryPlanner;
import ab.vision.ABObject;
import ab.vision.ABType;
import at.ac.tuwien.kr.alpha.Predicate;
import at.ac.tuwien.kr.alpha.common.terms.ConstantTerm;
import org.jbox2d.callbacks.RayCastCallback;
import org.jbox2d.collision.WorldManifold;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;

import java.awt.*;
import java.util.*;
import java.util.List;

import static java.util.Arrays.asList;
import static java.util.Collections.singletonList;
import static plugin.Config.POSITION_ITERATIONS;
import static plugin.Config.TIME_STEP;
import static plugin.Config.VELOCITY_ITERATIONS;

public class PredicateInterpretations {
	@Predicate(name = "on_top_all")
	public static Set<List<ConstantTerm>> onTopAll(ASPWorld aspWorld) {
		// TODO: Old interpretation would first hash the world and see if this was already computed.

		Set<List<ConstantTerm>> result = new LinkedHashSet<>();

		// TODO: Maybe scale factor should be a constant.
		World world = aspWorld.toWorld(10, true);
		
		/*

		if (debug) {
			DebugImage before("ontop-before.svg");
			before.DrawWorld(world, -1, SCALE_FACTOR);
			before.Save();
		}

		*/

		// Now we run the simulation until everything has settled.
		// Box2D sets bodys asleep after they have been idle for a short amount of time.
		// We loop the simulation until all objects have fallen asleep.
		boolean allAsleep = false;
		while (!allAsleep) {
			world.step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);

			allAsleep = true;

			for (Body b = world.getBodyList(); b != null; b = b.getNext()) {
				//After each step, we set the speed of the objects to zero, because otherwise they would gain a lot of speed, crash in the ground and explode a little, thereby destroying a lot of similarity between original objects (in the game world) and the objects used for calculations. We want the objects to settle very slow.
				b.setLinearVelocity(new Vec2(0, 0));
				b.setAngularVelocity(0);

				if (b.isAwake() && b.getType() == BodyType.DYNAMIC) {
					allAsleep = false;
				}
			}
		}

		//Contacts can only be calculated while objects are awake.
		world.setAllowSleep(false);

		//Final step to wake all objects.
		world.step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);

		//Now we can extract all the contacts from the world, filter them, and add them to our little graph.
		for (Contact c = world.getContactList(); c != null; c = c.getNext()) {
			//Filter contacts, that do not constitute a touch. Contacts are created as soon as two objects axis aligned bounding rectangles overlap. It is sometimes the case that the rectangles overlap while the objects are not touching (yet).
			if (!c.isTouching())
				continue;

			//Extract the object ids.
			int a = ASPWorld.getId(c.getFixtureA().getBody());
			int b = ASPWorld.getId(c.getFixtureB().getBody());

			//If either of the objects is a static objects, which get their user data filled with NULL, we can filter the contact, because we don't reason about the floor.
			if (a == -1 || b == -1)
				continue;

			//We need to use the world manifold, for the direction of the normal may be in some local coordinate system and therefore incorrect.
			WorldManifold worldManifold = new WorldManifold();
			c.getWorldManifold(worldManifold);

			//If the x part of the contact normal is not 0, the touch is not on top or bottom, but on the side. We do not consider these. Based on observations, even if one object is on top of a slightly inclined object, the contact normal will be straight up, so we can disregard any normals with x != 0 (>0.1 for floating point errors).
			if (worldManifold.normal.x > 0.1)
				continue;

			//The normal vector points from body a to body b, and we want the on_top_of(a,b)
			//relation to be true, if a is on top of b. If an object is nearer to the ground, and
			//therefore the bottom of the picture, it has a higher x value. Therefore the vector
			//a . b needs to be positive, so that it points from the higher to the lower object.
			//Only then can we use on_top_of(a,b). Otherwise, the object b is actually on top of a.
			if (worldManifold.normal.y > 0) {
				result.add(asList(
					ConstantTerm.getInstance(a),
					ConstantTerm.getInstance(b)
				));
			} else {
				result.add(asList(
					ConstantTerm.getInstance(b),
					ConstantTerm.getInstance(a)
				));
			}
		}

		/*

		if (debug) {
			DebugImage after("ontop-after.svg");
			after.DrawWorld(world, -1, SCALE_FACTOR);
			after.DrawGraph(graph, world, SCALE_FACTOR);
			after.Save();
		}

		 */

		return result;
	}

	@Predicate(name = "canpush")
	public Set<List<ConstantTerm>> canPush(ASPWorld aspWorld) {
		Set<List<ConstantTerm>> result = new LinkedHashSet<>();

		for (ABObject o : aspWorld.objects) {
			if (o.type.equals(ABType.Ground)) {
				continue;
			}
			
			for (ABObject i : aspWorld.objects) {
				if (o.id == i.id) {
					continue;
				}

				//
				// Angle penalty: should  we assume a too much inclined object is not that lean to push anything?
				//
				//const float angle_threshold = 30;
				Vec2 bottomO = Util.getBottomFaceCenter(o);
				Vec2 rightO = Util.getRightFaceCenter(o);

				Vec2 topI = Util.getTopFaceCenter(i);
				Vec2 bottomI = Util.getBottomFaceCenter(i);
				Vec2 leftI = Util.getLeftFaceCenter(i);
				//
				//  Assumes a rotation of 90° degrees clock wise of object o
				//
				float rotatedX = bottomO.x + o.height + o.width / 2;

				//std::cerr << "rightO.x <= leftI.x &&  rotatedX > leftI.x && bottomO.y >= topI.y && bottomO.y <= bottomI.y " << std::endl;
				//std::cerr << o.h << ":" << o.w << ":" << o.angle << "|" << rightO.x << ":" << leftI.x << ":" << rotatedX << ":" <<leftI.x << ":" <<bottomO.y << ":" <<topI.y << ":" <<bottomO.y << ":" <<bottomI.y << std::endl;

				if (rightO.x <= leftI.x && rotatedX > leftI.x
					&& bottomO.y >= topI.y && bottomO.y <= bottomI.y) {

					result.add(asList(
						ConstantTerm.getInstance(o.id),
						ConstantTerm.getInstance(i.id)
					));
				}
			}
		}

		return result;
	}

	@Predicate(name = "firstbelow")
	public static Set<List<ConstantTerm>> firstBelow(int oid, ASPWorld aspWorld) {
		Set<List<ConstantTerm>> result = new LinkedHashSet<>();

		if (oid < 0 || aspWorld.objects.size() == 0) {
			return result;
		}

		ABObject o = aspWorld.getById(oid);

		World world = aspWorld.toWorld(1, false, oid);

		Vec2 start = Util.pointToVec2(o.getCenter());
		Vec2 end = new Vec2(start.x, start.y + 600);

		FirstHit fh = new FirstHit();

		world.raycast(fh, start, end);

		Fixture fixture = fh.getHit();

		if (fixture == null) {
			return result;
		}

		result.add(singletonList(
			ConstantTerm.getInstance(ASPWorld.getId(fh.getHit().getBody()))
		));

		return result;
	}

	public static Set<List<ConstantTerm>> next(int oid, String targetHittingOffset, String trajectoryIdentifier, String velocity, Rectangle sling, ASPWorld aspWorld) {
		Set<List<ConstantTerm>> result = new LinkedHashSet<>();

		ABObject target = aspWorld.getById(oid);


		if (target.id < 0 || target.getType().equals(ABType.Ground)) {
			return result;
		}

		final boolean highTrajectory = trajectoryIdentifier.equals("high");

		Vec2 realTarget = highTrajectory ? Util.getTopFaceCenter(target) : Util.getLeftFaceCenter(target);

		if (highTrajectory) {
			realTarget.x = Float.parseFloat(targetHittingOffset);
		} else {
			realTarget.y = Float.parseFloat(targetHittingOffset);
		}

		TrajectoryPlanner tp = new TrajectoryPlanner();

		// TODO: The C++ version would receive the velcity as a third a parameter. Does it still work?
		List<Point> launchPoints = tp.estimateLaunchPoint(sling, Util.vec2ToPoint(realTarget));

		// If only one trajectory is possible, it is always the lower one.
		if (launchPoints.size() == 1 && highTrajectory) {
			return result;
		}

		final Point launchPoint = launchPoints.get(highTrajectory ? 1 : 0);

		// Now we create the world, scale factor 1 because we don't do any physics calculation anyways.
		World world = aspWorld.toWorld(1, false, Integer.MIN_VALUE);

		//float xMax = ASPWorld.maxXPlusW(world);

		// TODO: The C++ version would receive xMax and velocity as additional parameters. Does it still work?
		List<Point> trajectory = tp.predictTrajectory(sling, launchPoint);

		trajectory = Util.simplify(trajectory);

		// Now we step through the points of the trajectory one by one and check
		// if any point intersects with an object. If so, we add the object to the result vector.

		int count = 0;
		for (int i = 0; i < trajectory.size() - 1; i++) {
			final Point from = trajectory.get(i);
			final Point to = trajectory.get(i + 1);

			final AllHits ah = new AllHits();

			world.raycast(ah, Util.pointToVec2(from), Util.pointToVec2(to));

			for (Map.Entry<Float, Fixture> hit : ah.getHits().entrySet()) {
				final int hoid = ASPWorld.getId(hit.getValue().getBody());

				if (hoid == -1) {
					return result;
				}

				result.add(asList(
					ConstantTerm.getInstance(count++),
					ConstantTerm.getInstance(hoid))
				);
			}
		}

		return result;
	}

	/**
	 * A raycast callback implementation that stores the first reported fixture and then
	 * attempts to clip the ray at this point.
	 */
	private static class FirstHit implements RayCastCallback {
		private Fixture hit;

		@Override
		public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction) {
			hit = fixture;
			return fraction;
		}

		public Fixture getHit() {
			return hit;
		}
	}

	/**
	 * A raycast callback implementation that stores all reported fixtures.
	 */
	private static class AllHits implements RayCastCallback {
		/**
		 * The hits reported by the ray cast. Note that though we try to minimize the number of objects
		 * reported behind floor, there might still be some of them in the map.
		 */
		private final Map<Float, Fixture> hits = new LinkedHashMap<>();

		@Override
		public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction) {
			hits.put(fraction, fixture);
			// In case the floor is hit we tell Box2D that henceforth we only
			// want objects before this floor hit. Box2D takes the return value of this
			// function, and afterwards only notifies intersections that have a lower fraction
			// than the returned one.
			return ASPWorld.getId(fixture.getBody()) == -1 ? fraction : 1;
		}

		public Map<Float,Fixture> getHits() {
			return hits;
		}
	}
}
