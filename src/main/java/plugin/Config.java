package plugin;

public class Config {
	// Defines the factor, by which objects from the dlv program will be scaled for physics calculations. The numbers will be divided by this constant.
	public static final int SCALE_FACTOR = 10;

	// The arguments of a simulation step.
	public static final float TIME_STEP = 1.0f / 30.0f;
	public static final int VELOCITY_ITERATIONS = 6;
	public static final int POSITION_ITERATIONS = 2;

	// Densities for mass calculation
	public static final int PIG_DENSITY = 1;
	public static final int ICE_DENSITY = 1;
	public static final int WOOD_DENSITY = 2;
	public static final int STONE_DENSITY = 3;
	public static final int TNT_DENSITY = 2;

	public static final int RED_SIZE = 10;
	public static final int YELLOW_SIZE = 14;
	public static final int BLUE_SIZE = 9;
	public static final int BLACK_SIZE = 18;
	public static final int WHITE_SIZE = 18;
}
