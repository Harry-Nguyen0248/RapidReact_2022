/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class LedValues {
    public enum FixedPalettePattern {

        RAINBOW__RAINBOW_PALETTE(-0.99), RAINBOW__PARTY_PALETTE(-0.97), RAINBOW__OCEAN_PALETTE(-0.95),
        RAINBOW__LAVE_PALETTE(-0.93), RAINBOW__FOREST_PALETTE(-0.91), RAINBOW_WITH_GLITTER_PATTERN(-0.89),
        CONFETTI(-0.87), SHOT__RED(-0.85), SHOT__BLUE(-0.83), SHOT__WHITE(-0.81), SINELON__RAINBOW_PALETTE(-0.79),
        SINELON__PARTY_PALETTE(-0.77), SINELON__OCEAN_PALETTE(-0.75), SINELON__LAVA_PALETTE(-0.73),
        SINELON__FOREST_PALETTE(-0.71), BEATS_PER_MINUTE__RAINBOW_PALETTE(-0.69),
        BEATS_PER_MINUTE__PARTY_PALETTE(-0.67), BEATS_PER_MINUTE__OCEAN_PALETTE(-0.65),
        BEATS_PER_MINUTE__LAVA_PALETTE(-0.63), BEATS_PER_MINUTE__FOREST_PALETTE(-0.61), FIRE__MEDIUM(-0.59),
        FIRE__LARGE(-0.57), TWINKLES__RAINBOW_PALETTE(-0.55), TWINKLES__PARTY_PALETTE(-0.53),
        TWINKLES__OCEAN_PALETTE(-0.51), TWINKLES__LAVA_PALETTE(-0.49), TWINKLES__FOREST_PALETTE(-0.47),
        COLOR_WAVES__RAINBOW_PALETTE(-0.45), COLOR_WAVES__PARTY_PALETTE(-0.43), COLOR_WAVES__OCEAN_PALETTE(-0.41),
        COLOR_WAVES__LAVA_PALETTE(-0.39), COLOR_WAVES__FOREST_PALETTE(-0.37), LARSON_SCANNER__RED(-0.35),
        LARSON_SCANNER__GRAY(-0.33), LIGHT_CHASE__RED(-0.31), LIGHT_CHASE__BLUE(-0.29), LIGHT_CHASE__GRAY(-0.27),
        HEARTBEAT__RED(-0.25), HEARTBEAT__BLUE(-0.23), HEARTBEAT__WHITE(-0.21), HEARTBEAT__GRAY(-0.19),
        BREATH__RED(-0.17), BREATH__BLUE(-0.15), BREATH__GRAY(-0.13), STROBE__RED(-0.11), STROBE__BLUE(-0.09),
        STROBE__GOLD(-0.07), STROBE__WHITE(-0.05);

        private final double value;

        FixedPalettePattern(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    public enum Colour1Pattern {

        END_TO_END_BLEND_TO_BLACK(-0.03), LARSON_SCANNER(-0.01), LIGHT_CHASE(0.01), HEARTBEAT_SLOW(0.03),
        HEARTBEAT_MEDIUM(0.05), HEARTBEAT_FAST(0.07), BREATH_SLOW(0.09), BREATH_FAST(0.11), SHOT(0.13), STROBE(0.15);

        private final double value;

        Colour1Pattern(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    public enum Colour2Pattern {

        END_TO_END_BLEND_TO_BLACK(0.17), LARSON_SCANNER(0.19), LIGHT_CHASE(0.21), HEARTBEAT_SLOW(0.23),
        HEARTBEAT_MEDIUM(0.25), HEARTBEAT_FAST(0.27), BREATH_SLOW(0.29), BREATH_FAST(0.31), SHOT(0.33), STROBE(0.35);

        private final double value;

        Colour2Pattern(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    public enum Colour1And2Pattern {
        SPARKLE__COLOR_1_ON_COLOR_2(0.37), SPARKLE__COLOR_2_ON_COLOR_1(0.39), COLOR_GRADIENT__COLOR_1_AND_2(0.41),
        BEATS_PER_MINUTE__COLOR_1_AND_2(0.43), END_TO_END_BLEND__COLOR_1_TO_2(0.45), END_TO_END_BLEND(0.47),
        COLOR_1_AND_COLOR_2_NO_BLENDING(0.49), TWINKLES__COLOR_1_AND_2(0.51), COLOR_WAVES__COLOR_1_AND_2(0.53),
        SINELON__COLOR_1_AND_2(0.55);

        private final double value;

        Colour1And2Pattern(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    public enum SolidColour {
        HOT_PINK(0.57), DARK_RED(0.59), RED(0.61), RED_ORANGE(0.63), ORANGE(0.65), GOLD(0.67), YELLOW(0.69),
        LAWN_GREEN(0.71), LIME(0.73), DARK_GREEN(0.75), GREEN(0.77), BLUE_GREEN(0.79), AQUA(0.81), SKY_BLUE(0.83),
        DARK_BLUE(0.85), BLUE(0.87), BLUE_VIOLET(0.89), VIOLET(0.91), WHITE(0.93), GRAY(0.95), DARK_GRAY(0.97),
        BLACK(0.99);

        private final double value;

        SolidColour(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }
}
