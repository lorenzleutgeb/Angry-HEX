package plugin;

import org.jbox2d.common.Vec2;

public abstract class Xobject {
	protected float density = Float.NaN;

	int id;

	Vec2 center;
	Vec2 size;
	float angle;

	protected Xobject(int id, Vec2 center, Vec2 size, float angle, float density) {
		this.id = id;
		this.center = center;
		this.size = size;
		this.angle = angle;
		this.density = density;
	}

	public int getId() {
		return id;
	}

	public Vec2 getCenter() {
		return center;
	}

	public Vec2 getLeftFaceCenter() {
		return new Vec2(center.x - size.x / 2, center.y);
	}

	public Vec2 getTopFaceCenter() {
		return new Vec2(center.x, center.y - size.y / 2);
	}

	public Vec2 getRightFaceCenter() {
		return new Vec2(center.x + size.x / 2, center.y);
	}

	public Vec2 getBottomFaceCenter() {
		return new Vec2(center.x, center.y + size.y / 2);
	}

	void scale(float factor) {
		center = center.mul(1 / factor);
		size = size.mul(1 / factor);
	}

	float getMass() {
		return density * size.x * size.y;
	}
}
