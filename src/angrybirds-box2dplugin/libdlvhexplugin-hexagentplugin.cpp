class Calculation {
public:

		static void fillVector(
        const ComfortInterpretation& i,
        ASPWorld& world,
        const ComfortTerm& objectPredicate,
        const ComfortTerm& hillPredicate) {

    const bool debug = false;

    // we initially store the polygons like this because the atoms might come in any order
    //
    // key = index of vertex, value = x/y
    typedef std::map<int, std::pair<int, int> > PolyMap;
    // key = id of polygon, value = polygon
    std::map<int, PolyMap> polys;

		for (ComfortInterpretation::iterator c = i.begin(); c != i.end(); ++c) {
			} else if (c->getPredicate() == hillPredicate.getUnquotedString()) {
  			// handle polygon objects (hills)
				int id = c->getArgument(1).intval;
				int idx = c->getArgument(2).intval;
				int x = c->getArgument(3).intval;
				int y = c->getArgument(4).intval;
        
        // this creates an empty one if it does not exist
        PolyMap& poly = polys[id];
        // this creates an empthy pair if it does not exist
        poly[idx].first = x;
        poly[idx].second = y;
			} else {
				std::cerr << "Calculation::fillVector got part of interpretation for unexpected predicate '" << c->getPredicate() << "'" << std::endl;
      }
		}

    // create real polygons in aspworld from the map/map
    for(const auto& poly_id_v : polys) {
      // we ignore the polygon id
      const PolyMap& p = poly_id_v.second;
      std::vector<b2Vec2> vertices;
      if (debug) std::cerr << "Poly with " << p.size() << " vertices" << std::endl;
      for(unsigned idx = 0; idx < p.size(); ++idx) {
        // we do not use idx but p.size()-1-idx to make the polygon vertices arranged correctly
        // TODO this assumes that the polygons always come in convex and in wrong order!
        auto vtx = p.find(idx); // p.size()-1-idx);
        assert(vtx != p.end());
        float x = vtx->second.first;
        float y = vtx->second.second;
        if (debug) std::cerr << " vertex " << idx << " is " << x << "/" << y;
        vertices.push_back(b2Vec2(x, y));
      }
      if (debug) std::cerr << std::endl;
      // now we need to triangulate
      std::vector<std::vector<b2Vec2> > triangulated;
      triangulate(vertices, triangulated);
      for( auto tri : triangulated ) {
        world.hills.push_back(b2PolygonShape());
        world.hills.back().Set(&tri[0], tri.size());
        if (debug) { std::cerr << " tri"; for( unsigned u = 0; u < tri.size(); ++u ) std::cerr << " " << tri[u].x << "/" << tri[u].y; }
      }
      if (debug) std::cerr << std::endl;
    }
	}
};

class StabilityAtom: public ComfortPluginAtom {
public:

	StabilityAtom() :
			ComfortPluginAtom("stability") {
		setOutputArity(1);
		addInputConstant(); //object w
		addInputConstant(); //object h
		addInputConstant(); //ojbect angle
	}

	virtual void retrieve(const ComfortQuery& query, ComfortAnswer& answer) {
		Object o;
		o.w = ((ComfortTerm)(query.input[0])).intval;
		o.h = ((ComfortTerm)(query.input[1])).intval;
		o.angle = std::atof(
				((ComfortTerm)(query.input[2])).getUnquotedString().c_str());

		ComfortTuple ct;
		ct.push_back(ComfortTerm::createInteger(getStability(o)));
		answer.insert(ct);
	}

private:

	int getStability(const Object& o) {
		ASPWorld aspworld;
		aspworld.objects.push_back(o);
		b2World world = WorldCreation::createWorld(aspworld, 1);

		b2AABB rect = world.GetBodyList()->GetFixtureList()->GetAABB(0);

		float width = rect.upperBound.x - rect.lowerBound.x;
		float height = rect.upperBound.y - rect.lowerBound.y;

		return std::min((int) ((width / height) * 50.0), 100);
	}
};

class ClearSkyAtom: public ComfortPluginAtom {
public:

	ClearSkyAtom() :
			ComfortPluginAtom("clearsky") {
		setOutputArity(1);
		addInputConstant(); //object id
		addInputPredicate(); //objects
		addInputPredicate(); //hills
	}

	virtual void retrieve(const ComfortQuery& query, ComfortAnswer& answer) {
		// First we retrieve the object whose mass should be calculated.
		int oid = ((ComfortTerm)(query.input[0])).intval;

		ASPWorld world;
		Calculation::fillVector(query.interpretation, world, query.input[1].getUnquotedString(), query.input[2].getUnquotedString());
		if (world.objects.empty())
			std::cerr << "&firstbelow: Called with no objects." << std::endl;

		Object object = Object::findById(world.objects, oid);
		if (object.id == -1) {
			std::cerr << "&firstbelow: Called with invalid target id " << oid
					<< "." << std::endl;
			return;
		}

		b2Vec2 face = object.getTopFaceCenter();
		double width = object.w;
		int olx = object.getLeftFaceCenter().x;
		int orx = object.getRightFaceCenter().x;
		bool clearsky = true;
		const float OVERLAP_PERCENT = 0.7;
		if (debug)
			std::cerr << "OBJ:" << object.id << " " << width << std::endl;
		for (std::vector<Object>::const_iterator i = world.objects.begin();
				i != world.objects.end(); ++i) {
			b2Vec2 bot = i->getBottomFaceCenter();
			int lx = i->getLeftFaceCenter().x;
			int rx = i->getRightFaceCenter().x;
			if (debug)
				std::cerr << "CK:" << i->id << "=" << lx << ":" << rx << ":"
						<< olx << ":" << orx;
			double overlap = std::max(0,
					(-std::max(lx, olx) + std::min(rx, orx))) / width;
			if (debug)
				std::cerr << " OVL:" << overlap << " " << OVERLAP_PERCENT
						<< std::endl;
			if (bot.y < face.y && overlap >= OVERLAP_PERCENT) {
				clearsky = false;
				break;
			}
		}

		if (clearsky) {
			ComfortTuple ct;
			ct.push_back(ComfortTerm::createInteger(1));
			answer.insert(ct);
		}
	}

};

class ShootableAtom: public ComfortPluginAtom {
public:

	ShootableAtom() :
			ComfortPluginAtom("shootable") {
		setOutputArity(3);
		addInputConstant(); //target id
		addInputConstant(); //trajectory identifier, low or high
		addInputConstant(); //velocity
		addInputConstant(); //slingshot x
		addInputConstant(); //slingshot y
		addInputConstant(); //slingshot width
		addInputConstant(); //slingshot height
		addInputConstant(); // birdtype
		addInputPredicate(); //objects
		addInputPredicate(); //hills
	}

	virtual void retrieve(const ComfortQuery& query, ComfortAnswer& answer) {
		// First we retrieve the object whose mass should be calculated.
		int targetid = ((ComfortTerm)(query.input[0])).intval;
		std::string trajectory =
				((ComfortTerm)(query.input[1])).getUnquotedString();
		double velocity = std::atof(
				((ComfortTerm)(query.input[2])).getUnquotedString().c_str());

		Sling slingshot;

		slingshot.x = ((ComfortTerm)(query.input[3])).intval;
		slingshot.y = ((ComfortTerm)(query.input[4])).intval;
		slingshot.w = ((ComfortTerm)(query.input[5])).intval;
		slingshot.h = ((ComfortTerm)(query.input[6])).intval;

		std::string birdType =
				((ComfortTerm)(query.input[7])).getUnquotedString();
		int size;
		if (birdType == "redbird")
			size = RED_SIZE;
		else if (birdType == "yellowbird")
			size = YELLOW_SIZE;
		else if (birdType == "bluebird")
			size = BLUE_SIZE;
		else if (birdType == "blackbird")
			size = BLACK_SIZE;
		else if (birdType == "whitebird")
			size = WHITE_SIZE;

		ASPWorld world;
		Calculation::fillVector(query.interpretation, world, query.input[8], query.input[9]);
		if (world.objects.empty())
			std::cerr << "&next: Called with no objects." << std::endl;

		Object target = Object::findById(world.objects, targetid);
		if (target.id == -1) {
			std::cerr << "&next: Called with invalid target id " << targetid
					<< "." << std::endl;
			return;
		}

		//Ground is not shootable and nothing can be hit behind it.
		if (target.type == ground)
			return;

		const int POINTS = 9;
		const int GUARD = 2;
		int c[POINTS];
		b2Vec2 p[POINTS];

		b2Vec2 left = target.getLeftFaceCenter();
		b2Vec2 top = target.getTopFaceCenter();
		b2Vec2 right = target.getRightFaceCenter();
		b2Vec2 bottom = target.getBottomFaceCenter();

		if (trajectory == "high") {
			p[0] = top;
			for (int i = 1; i < POINTS; i++)
				p[i] = p[0];
			//
			// p[2] is middle
			//
			// Group 1
			p[0].x = (int) left.x + GUARD;
			p[1].x = (int) std::min(left.x + GUARD + size / 2, right.x);
			p[2].x = (int) std::min(left.x + GUARD + size, right.x);

			// Group 2
			p[3].x = (int) std::max(top.x - size / 2, left.x);
			p[4].x = (int) top.x;
			p[5].x = (int) std::min(top.x + size / 2, right.x);

			// Group 3
			p[6].x = (int) std::max(right.x - GUARD - size, left.x);
			;
			p[7].x = (int) std::max(right.x - GUARD - size / 2, left.x);
			p[8].x = (int) right.x - GUARD;

		} else //  low trajectory. Alternatives are computed from the middle point upwards.
		{

			p[0] = left;
			for (int i = 1; i < POINTS; i++)
				p[i] = p[0];

			// Group 1
			p[0].y = (int) (bottom.y - GUARD);
			p[1].y = (int) std::max(bottom.y - GUARD - size / 2, top.y);
			p[2].y = (int) std::max(bottom.y - GUARD - size, top.y);
			;

			// Group 2
			p[3].y = (int) std::max(left.y - size / 2, top.y);
			p[4].y = (int) left.y;
			p[5].y = (int) std::min(left.y + size / 2, bottom.y);
			;

			// Group 3
			p[6].y = (int) std::min(top.y + GUARD + size, bottom.y);
			p[7].y = (int) std::min(top.y + GUARD + size / 2, bottom.y);
			p[8].y = (int) (top.y + GUARD);

		}

		if (debug)
			std::cerr << "Object:" << targetid << " -> " << target.cx << ":"
					<< target.cy << ":" << target.w << ":" << target.h
					<< std::endl;

		if (debug) {
			std::cerr << (trajectory == "high" ? "H=" : "L=");
			for (int i = 0; i < POINTS; i++)
				std::cerr << p[i].x << ":" << p[i].y << " ";
			std::cerr << std::endl;
		}

		for (int i = 0; i < POINTS; i++) {
			if (debug)
				std::cerr << "GFO:" << targetid << " " << p[i].x << ":"
						<< p[i].y;

			c[i] = getFrontObject(p[i], trajectory, slingshot, velocity, world);
			if (debug)
				std::cerr << "==>" << c[i] << std::endl;
		}
		if (debug)
			std::cerr << std::endl;

		int shift = -1;
		int retValues[POINTS / 3];

		for (int i = 0; i < POINTS / 3; i++)
			retValues[i] = -1;

		for (int i = 0; i < POINTS / 3; i++)

			if (targetid == c[i * 3] && c[i * 3] == c[i * 3 + 1]
					&& c[i * 3] == c[i * 3 + 2]) {
				retValues[i] = shift = (
						trajectory == "high" ? p[i * 3 + 1].x : p[i * 3 + 1].y);
				if (debug)
					std::cerr << "TP:" << i << "=" << retValues[i] << std::endl;
			}

		if (debug)
			std::cerr << std::endl;
		//
		// shift will contain in priority order, either center, upwards or downwards point.
		// For low trajectories the downwards point is the middle of left face
		// For high trajectories downwards is meant to be leftmost Y point
		//
		// The worst point of the three (the downward one) is never returned unless it is the unique
		//
		bool targetHit = false;
		for (int i = 0; i < POINTS; i++) {
			targetHit = targetHit || c[i] == targetid;
		}

		if (targetHit && shift >= 0) {
			ComfortTuple ct;
			ct.push_back(ComfortTerm::createInteger(targetid));

			//
			//  For low trajectories we propose center and upper trajectory first (the third one only if none is avail
			//
			if (trajectory == "low") {
				ct.push_back(
						ComfortTerm::createInteger(
								retValues[POINTS / 3 - 2] >= 0 ?
										retValues[POINTS / 3 - 2] : shift));
				ct.push_back(
						ComfortTerm::createInteger(
								retValues[POINTS / 3 - 1] >= 0 ?
										retValues[POINTS / 3 - 1] : shift));
			}
			//
			//  For high trajectories we propose center and leftward trajectory first (the third one only if none is avail
			//

			else {
				ct.push_back(
						ComfortTerm::createInteger(
								retValues[POINTS / 3 - 2] >= 0 ?
										retValues[POINTS / 3 - 2] : shift));
				ct.push_back(
						ComfortTerm::createInteger(
								retValues[POINTS / 3 - 3] >= 0 ?
										retValues[POINTS / 3 - 3] : shift));

			}
			//std::cerr << "RETURNING " << c[0] << " " << retValues[2] << shift << " " << targetid << std::endl;
			answer.insert(ct);
		}
	}

private:
	class RCCB: public b2RayCastCallback {
	public:
		///The hits reported by the ray cast. Key is a float for sorting in hit order, value is the object id. Note that though we try to minimize the number of objects reported behind floor, there might still be some of them in the map.
		std::map<float32, int> hits;

		virtual float32 ReportFixture(b2Fixture* fixture, const b2Vec2&,
				const b2Vec2&, float32 fraction) {
			int id = WorldCreation::GetId(fixture->GetBody());

			hits[fraction] = id;

			//In this case (when the floor is hit) we tell Box2D that henceforth we only want objects before this floor hit. Box2D takes the return value of this function, and afterwards only notifies intersections that have a lower fraction than the returned one.
			if (id == -1)
				return fraction;

			return 1;
		}
	};

	/**
	 * Gets front object in a given trajectory.
	 */

	static int getFrontObject(const b2Vec2& hitPoint,
			const std::string& trajectory, const Sling& slingshot,
			const double& velocity, ASPWorld& aspworld) {
		//First, we calculate the two possible release points (if there are two).
		std::vector < b2Vec2 > launchPoints =
				TrajectoryPlanner::estimateLaunchPoint(slingshot, hitPoint,
						velocity);

		//If only one trajectory is possible, it is always the lower one.
		if (launchPoints.size() == 1 && trajectory == "high")
			return -3;

		//Extracting the correct launch point.
		b2Vec2 launchPoint;
		if (trajectory == "high")
			launchPoint = launchPoints[1];
		else
			launchPoint = launchPoints[0];

		//Now we create the world, scale factor 1 because we don't do any physics calculation anyways.
		b2World world = WorldCreation::createWorld(aspworld, 1);

		//And we calculate all the points in the trajectory.
		int x_max = WorldCreation::maxXPlusW(world);
		std::vector < b2Vec2 > traj = TrajectoryPlanner::predictTrajectory(
				slingshot, launchPoint, x_max, velocity);

		traj = simplifyTrajectory(traj);

		//Now we fill the result vector. We step through the points of the trajectory one by one and check if any point intersects with an object. If so, we add the object to the result vector.
		std::vector<int> result;

		for (std::vector<b2Vec2>::iterator i = traj.begin();
				i + 1 != traj.end(); ++i) {
			RCCB cb;

			world.RayCast(&cb, *i, *(i + 1));

			for (std::map<float32, int>::const_iterator hit = cb.hits.begin();
					hit != cb.hits.end(); ++hit) {
				if ((*hit).second == -1) {

					return -1;
				}

				return ((*hit).second);
			}
		}
		return -2;
	}


};

/*
 *    Takes a target and finds the lowest usable trajectory for a white bird iterating from the object y up in 10 pixel steps.
 */
class BestWhiteAtom: public ComfortPluginAtom {
public:

	BestWhiteAtom() :
			ComfortPluginAtom("bestwhite") {
		setOutputArity(1);
		addInputConstant(); //target id
		addInputConstant(); //trajectory identifier, low or high
		addInputConstant(); //velocity
		addInputConstant(); //slingshot x
		addInputConstant(); //slingshot y
		addInputConstant(); //slingshot width
		addInputConstant(); //slingshot height
		addInputPredicate(); //objects
		addInputPredicate(); //hills
	}

	virtual void retrieve(const ComfortQuery& query, ComfortAnswer& answer) {
		// First we retrieve the object whose mass should be calculated.
		int targetid = ((ComfortTerm)(query.input[0])).intval;
		std::string trajectory =
				((ComfortTerm)(query.input[1])).getUnquotedString();
		double velocity = std::atof(
				((ComfortTerm)(query.input[2])).getUnquotedString().c_str());

		Sling slingshot;

		slingshot.x = ((ComfortTerm)(query.input[3])).intval;
		slingshot.y = ((ComfortTerm)(query.input[4])).intval;
		slingshot.w = ((ComfortTerm)(query.input[5])).intval;
		slingshot.h = ((ComfortTerm)(query.input[6])).intval;

    /*
       TODO at the moment we do not pass a bird type!
		std::string birdType =
				((ComfortTerm)(query.input[7])).getUnquotedString();
		int size;
		if (birdType == "redbird")
			size = RED_SIZE;
		else if (birdType == "yellowbird")
			size = YELLOW_SIZE;
		else if (birdType == "bluebird")
			size = BLUE_SIZE;
		else if (birdType == "blackbird")
			size = BLACK_SIZE;
		else if (birdType == "whitebird")
			size = WHITE_SIZE;
      */
    // TODO size was an uninitialized value before, because birdType="objects" was always used
		int size = WHITE_SIZE;

		ASPWorld world;

		Calculation::fillVector(query.interpretation, world, query.input[7], query.input[8]);
		if (world.objects.empty())
			std::cerr << "&next: Called with no objects." << std::endl;

		Object target = Object::findById(world.objects, targetid);
		if (target.id == -1) {
			std::cerr << "&next: Called with invalid target id " << targetid
					<< "." << std::endl;
			return;
		}

		//Ground is not shootable and nothing can be hit behind it.
		if (target.type == ground)
			return;

		const int POINTS = 3;
		const int GUARD = 2;
		int c[POINTS];
		Object ob[POINTS];
		b2Vec2 p[POINTS];

		b2Vec2 left = target.getLeftFaceCenter();
		b2Vec2 top = target.getTopFaceCenter();
		b2Vec2 right = target.getRightFaceCenter();
		b2Vec2 bottom = target.getBottomFaceCenter();

		int shift = -1;
		int X_THRESHOLD = top.x + 20;
		for (p[0].y = top.y - 10; p[0].y >= 40; p[0].y -= 20) {

			for (int i = 1; i < POINTS; i++)
				p[i] = p[0];

			p[0].x = (int) std::max(top.x - size / 2, left.x);
			p[1].x = (int) top.x;
			p[2].x = (int) std::min(top.x + size / 2, right.x);

			if (debug)
				std::cerr << "Object:" << targetid << " -> " << target.cx << ":"
						<< target.cy << ":" << target.w << ":" << target.h
						<< std::endl;

			if (debug) {
				std::cerr << (trajectory == "high" ? "H=" : "L=");
				for (int i = 0; i < POINTS; i++)
					std::cerr << p[i].x << ":" << p[i].y << " ";
				std::cerr << std::endl;
			}

			for (int i = 0; i < POINTS; i++) {
				if (debug)
					std::cerr << "BW:" << targetid << " " << p[i].x << ":"
							<< p[i].y;

				c[i] = getFrontObject(p[i], trajectory, slingshot, velocity, world);
				ob[i] = Object::findById(world.objects, c[i]);

				if (debug)
					std::cerr << "==>" << c[i] << std::endl;
			}
			if (debug)
				std::cerr << std::endl;

			shift = -1;

			if (ob[0].getLeftFaceCenter().x > X_THRESHOLD
					&& ob[0].getLeftFaceCenter().x > X_THRESHOLD
					&& ob[0].getLeftFaceCenter().x > X_THRESHOLD) {
				shift = p[1].y;
				if (debug)
					std::cerr << "TP << " << shift << std::endl;
				break;
			}

		} // for
		if (debug)
			std::cerr << std::endl;

		if (shift >= 0) {
			ComfortTuple ct;
			ct.push_back(ComfortTerm::createInteger(shift));
			answer.insert(ct);
		}
	}

private:
	/**
	 * Gets front object in a given trajectory.
	 */

	static int getFrontObject(const b2Vec2& hitPoint,
			const std::string& trajectory, const Sling& slingshot,
			const double& velocity, ASPWorld& aspworld) {
		//First, we calculate the two possible release points (if there are two).
		std::vector < b2Vec2 > launchPoints =
				TrajectoryPlanner::estimateLaunchPoint(slingshot, hitPoint,
						velocity);

		//If only one trajectory is possible, it is always the lower one.
		if (launchPoints.size() == 1 && trajectory == "high")
			return -3;

		//Extracting the correct launch point.
		b2Vec2 launchPoint;
		if (trajectory == "high")
			launchPoint = launchPoints[1];
		else
			launchPoint = launchPoints[0];

		//Now we create the world, scale factor 1 because we don't do any physics calculation anyways.
		b2World world = WorldCreation::createWorld(aspworld, 1);

		//And we calculate all the points in the trajectory.
		int x_max = WorldCreation::maxXPlusW(world);
		std::vector < b2Vec2 > traj = TrajectoryPlanner::predictTrajectory(
				slingshot, launchPoint, x_max, velocity);

		traj = simplifyTrajectory(traj);

		//Now we fill the result vector. We step through the points of the trajectory one by one and check if any point intersects with an object. If so, we add the object to the result vector.
		std::vector<int> result;

		for (std::vector<b2Vec2>::iterator i = traj.begin();
				i + 1 != traj.end(); ++i) {

			// This is an AllHits RCCB
			RCCB cb;

			world.RayCast(&cb, *i, *(i + 1));

			for (std::map<float32, int>::const_iterator hit = cb.hits.begin();
					hit != cb.hits.end(); ++hit) {
				if ((*hit).second == -1) {

					return -1;
				}

				return ((*hit).second);
			}
		}
		return -2;
	}
};