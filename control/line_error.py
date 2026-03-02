
# Weights for 4 sensors [L, ML, MR, R]
# Negative = line on left, positive = line on right
WEIGHTS = [-1.5, -0.5, +0.5, +1.5]

# Compute line position error from blackness list [L, ML, MR, R]
# black: list of 4 ints (0/1), where 1 = black line detected
# Returns:
#   error < 0  => line is left
#   error > 0  => line is right
#   error = 0  => centred
#   None       => line lost (sum == 0)
def error_from_black(black, weights=WEIGHTS):
    s = black[0] + black[1] + black[2] + black[3]
    if s == 0:
        return None

    num = (
        weights[0] * black[0] +
        weights[1] * black[1] +
        weights[2] * black[2] +
        weights[3] * black[3]
    )
    return num / s