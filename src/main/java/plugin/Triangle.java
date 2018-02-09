package plugin;

import org.jbox2d.common.Vec2;

public class Triangle {
	public static final double EPSILON = 0.000001;

	private final Vec2 a;
	private final Vec2 b;
	private final Vec2 c;

	public Triangle(final Vec2 a, final Vec2 b, final Vec2 c) {
		this.a = a;
		this.b = b;
		this.c = c;
	}

	public double area()
	{
		return Math.abs((
			a.x * (b.y - c.y) +
			b.x * (c.y - a.y) +
			c.x * (a.y - b.y)
		) / 2.0);
	}

	public boolean inside(Vec2 p) {
		double area = area();

		double A1 = (new Triangle(p, b, c)).area();
		double A2 = (new Triangle(p, a, c)).area();
		double A3 = (new Triangle(p, a, b)).area();

		return (area - A1 + A2 + A3 < EPSILON);
	}
}
