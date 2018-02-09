package plugin;

import ab.vision.ABObject;
import org.jbox2d.common.Vec2;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public abstract class Util {
	public static Vec2 pointToVec2(Point point) {
		return new Vec2(point.x, point.y);
	}

	public static Point vec2ToPoint(Vec2 vec) {
		return new Point(Math.round(vec.x), Math.round(vec.y));
	}

	public static Vec2 getLeftFaceCenter(ABObject o) {
		return new Vec2((float) o.getCenterX() - o.width / 2, (float) o.getCenterY());
	}

	public static Vec2 getTopFaceCenter(ABObject o) {
		return new Vec2((float) o.getCenterX(), (float) o.getCenterY() - o.height / 2);
	}

	public static Vec2 getRightFaceCenter(ABObject o) {
		return new Vec2((float) o.getCenterX() + o.width / 2, (float) o.getCenterY());
	}

	public static Vec2 getBottomFaceCenter(ABObject o) {
		 return new Vec2((float) o.getCenterX(), (float) o.getCenterY() + o.height / 2);
	}

	public static <T> List<T> simplify(List<T> trajectory) {
		// We initilize the vector to size/10, the number of vertices that we will keep,
		// plus one for the last vertex.
		final List<T> result = new ArrayList<>(trajectory.size() / 10 + 1);


		for (int i = 0; i < trajectory.size(); i+= 10) {
			result.add(trajectory.get(i));

			if (i == trajectory.size() - 1) {
				// assert trajectory.size() / 10 + 1 < result.capacity()
				return result;
			}
		}

		result.add(trajectory.get(trajectory.size() - 1));

		// Make sure there have not been any resize operations.
		// If there had been any, we would need to readjust our
		// starting values.
		// assert trajectory.size() / 10 + 1 < result.max_size()

		return result;
	}
}
