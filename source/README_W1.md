This week I achieved having the grid solver to work with ghost cells.
This is what needs to be investiated:
    - Increase locality per particle over surrounding cells
        - Outer current cell id to inner in cells ?
    - proceed symetrically in cell => for all particle in current cell also add the corresponding quantity to the cells in the lower
    triangular half of the 9th cube.
    - sort particles when having computed thier neighbors id
    - investigate the use of Z index sort and related method
    - maybe sort every 10th frame

Idea of a fluid game:
    - you play a fluid
    - two poeple are playing
    - The map is made with obstacle and this is a death round
    - Upon contact between fluids, one will "eat" the other depending on some intuitive rule