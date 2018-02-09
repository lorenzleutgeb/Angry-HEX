package plugin;

import org.jbox2d.common.Vec2;

import java.util.Stack;

public class Contour {
	public static final double EPSILON = 0.000001;

	private final Vec2[] points;

	public Contour(Vec2... points) {
		this.points = points;
	}

	public double area() {
		double a = 0.0d;
		for (int i = points.length - 1, j = 0; j < points.length; i = j++) {
			a += points[i].x * points[j].y - points[j].x * points[i].y;
		}
		return a * 0.5;
	}

	public Triangle[] triangulate() {
		int n = points.length;
		if (n < 3) return null;

		Stack<Triangle> result = new Stack<>();

		int[] V = new int[n];

		/* we want a counter-clockwise polygon in V */

		if (0.0f < area())
			for (int v = 0; v < n; v++) V[v] = v;
		else
			for (int v = 0; v < n; v++) V[v] = (n - 1) - v;

		int nv = n;

		/*  remove nv-2 Vertices, creating 1 triangle every time */
		int count = 2 * nv;   /* error detection */

		for (int m = 0, v = nv - 1; nv > 2; ) {
			/* if we loop, it is probably a non-simple polygon */
			if (0 >= (count--)) {
				//** Triangulate: ERROR - probable bad polygon!
				return null;
			}

			/* three consecutive vertices in current polygon, <u,v,w> */
			int u = v;
			if (nv <= u) u = 0;     /* previous */
			v = u + 1;
			if (nv <= v) v = 0;     /* new v    */
			int w = v + 1;
			if (nv <= w) w = 0;     /* next     */

			if (snip(u, v, w, nv, V)) {
				int a, b, c, s, t;

				/* true names of the vertices */
				a = V[u];
				b = V[v];
				c = V[w];

				/* output Triangle */
				result.push(new Triangle(points[a], points[b], points[c]));

				m++;

				/* remove v from remaining polygon */
				for (s = v, t = v + 1; t < nv; s++, t++) V[s] = V[t];
				nv--;

				/* resest error detection counter */
				count = 2 * nv;
			}
		}

		return result.toArray(new Triangle[result.size()]);
	}

	private boolean snip(int u, int v, int w, int n, int[] V) {
		final Vec2 a = points[V[u]];
		final Vec2 b = points[V[v]];
		final Vec2 c = points[V[w]];

		if (EPSILON > (((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x)))) {
			return false;
		}

		for (int i = 0; i < n; i++) {
			if ((i == u) || (i == v) || (i == w)) {
				continue;
			}
			if ((new Triangle(a, b, c)).inside(points[V[i]])) {
				return false;
			}
		}

		return true;
	}
}
