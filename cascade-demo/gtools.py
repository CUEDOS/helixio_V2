from math import floor


def alt_calc(alt_dict):
    """
    Parameters
    ------------
    alt_dict: Dict(key drone_index (string): value altitude (float), ...)

    Returns
    -----------
    output_dict: Dict(key:drone_index (string), value: altitude (float))
    """

    # sorted_idx = sorted(alt_dict, key=alt_dict.get)
    # gives a list of IDs in order from smallest altitude to largest

    min_alt = 10  # in meters - min return altitude above launch altitude
    alt_step = 2  # in meters - the alt difference between return alts

    # Saving current altitudes in alts array -----------------------------------------------------------------------------
    alts = [0] * CONST_SWARM_SIZE
    for i in range(0, CONST_SWARM_SIZE):
        alts[i] = alt_dict["P" + str(i + 101)]

    alts.sort()  # Sorting alts

    # Calculating the mean of current alts -------------------------------------------------------------------------------
    mean = 0
    for i in range(0, CONST_SWARM_SIZE):
        mean = mean + alts[i]
    mean = mean / CONST_SWARM_SIZE

    # Creating sorted return alts ------------------------------------------------------------------------------------------
    middle_element = floor(CONST_SWARM_SIZE / 2)
    alt_return_sorted = [0] * CONST_SWARM_SIZE
    alt_return_sorted[middle_element] = mean

    if CONST_SWARM_SIZE % 2 == 1:
        for i in range(1, int((CONST_SWARM_SIZE - 1) / 2) + 1):
            alt_return_sorted[middle_element - i] = mean - i * alt_step
            alt_return_sorted[middle_element + i] = mean + i * alt_step
    else:
        for i in range(1, int((CONST_SWARM_SIZE - 2) / 2) + 1):
            alt_return_sorted[middle_element - i] = mean - i * alt_step
            alt_return_sorted[middle_element + i] = mean + i * alt_step
        alt_return_sorted[0] = alt_return_sorted[1] - alt_step

    # Checking minimum alt -----------------------------------------------------------------------------------------------
    if alt_return_sorted[0] < min_alt:
        difference = min_alt - alt_return_sorted[0]
        for i in range(0, CONST_SWARM_SIZE):
            alt_return_sorted[i] = alt_return_sorted[i] + difference

    # assigning sorted alts (Now i shows the order of the drones in alts) -----------------------------------------------
    alt_return_dict=alt_dict
    for i in range(0, CONST_SWARM_SIZE):
        for j in range(0, CONST_SWARM_SIZE ):
            if alt_dict["P" + str(j + 101)] == alts[i]: # drone "P" + str(j + 101) is the ith drone 
                alt_return_dict["P" + str(j + 101)]= alt_return_sorted[i]

    return alt_return_dict
