package org.firstinspires.ftc.teamcode.sim.biobuzz;

/**
 * Live score for one alliance, using BIOBUZZ Table 10-2 values.
 * Not included: LEAVE, PARK, penalties, and the "flowers only score in the last minute" rule.
 */
public class Score {
    public int tips;
    public int piecesInCells;
    public int flowersOwned;
    public int piecesInOwnedFlowers;
    public int bottomNectarBonuses;
    public int piecesInGarden;

    public int total() {
        return tips * 20
                + piecesInCells * 2
                + piecesInOwnedFlowers * 2
                + bottomNectarBonuses * 5
                + piecesInGarden;
    }

    @Override
    public String toString() {
        return String.format("%d pts (tips %d, in cells %d, flowers owned %d / %d pieces, bottom-nectar %d, garden %d)",
                total(), tips, piecesInCells, flowersOwned, piecesInOwnedFlowers, bottomNectarBonuses, piecesInGarden);
    }
}
