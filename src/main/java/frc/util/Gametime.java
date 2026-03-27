package frc.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * Custom 2026 FRC Game Timer Widget
 * 
 * Match Timeline:
 * - Autonomous: 0-20 seconds
 * - Teleop Phase 1: 20-30 seconds (BOTH sides active)
 * - Teleop Phase 2-5: 30-110 seconds (20 second cycles, alternating sides)
 *   - Each cycle: 20 seconds active, 20 seconds inactive, then switch
 *   - CHANGE notifications every 20 seconds
 * - Endgame: Last 30 seconds (BOTH sides active)
 * 
 * Total Match Time: 2:30 (150 seconds)
 */
public class Gametime {
    
    // Match phases
    public enum GamePhase {
        AUTO,           // 0-20 seconds
        TELEOP_START,   // 20-30 seconds (both active)
        TELEOP_CYCLE_1, // 30-70 seconds (cycle 1)
        TELEOP_CYCLE_2, // 70-110 seconds (cycle 2)
        ENDGAME         // 120-150 seconds (last 30 seconds)
    }
    
    // Field states
    public enum FieldState {
        BOTH_ACTIVE,    // Both alliances can score
        RED_ACTIVE,     // Only red can score
        BLUE_ACTIVE     // Only blue can score
    }
    
    private static final double MATCH_LENGTH = 150.0; // 2:30
    private static final double AUTO_LENGTH = 20.0;
    private static final double TELEOP_START_LENGTH = 10.0; // 20-30 seconds
    private static final double CYCLE_LENGTH = 20.0;
    private static final double NUM_CYCLES = 4.0;
    private static final double ENDGAME_LENGTH = 30.0;
    
    private final Timer matchTimer;
    private final List<GameStateListener> listeners;
    private FieldState currentFieldState;
    private FieldState lastFieldState;
    private GamePhase currentPhase;
    private GamePhase lastPhase;
    private boolean changeNotificationTriggered;
    
    private final NetworkTable table;
    
    public interface GameStateListener {
        void onFieldStateChanged(FieldState newState, FieldState oldState);
        void onPhaseChanged(GamePhase newPhase, GamePhase oldPhase);
        void onChangeNotification(double timeUntilChange);
    }
    
    public Gametime() {
        this.matchTimer = new Timer();
        this.listeners = new ArrayList<>();
        this.currentFieldState = FieldState.BOTH_ACTIVE;
        this.lastFieldState = FieldState.BOTH_ACTIVE;
        this.currentPhase = GamePhase.AUTO;
        this.lastPhase = GamePhase.AUTO;
        this.changeNotificationTriggered = false;
        this.table = NetworkTableInstance.getDefault().getTable("Gametime2026");
    }
    
    /**
     * Call this periodically (every 20ms in robot code) to update game state
     */
    public void periodic() {
        if (!DriverStation.isEnabled()) {
            return;
        }
        
        double matchTime = DriverStation.getMatchTime();
        
        // Update phase
        updateGamePhase(matchTime);
        
        // Update field state based on phase and time
        updateFieldState(matchTime);
        
        // Check for change notifications
        checkForChangeNotification(matchTime);
        
        // Notify listeners if state changed
        notifyListenersIfChanged();
        
        // Update network tables
        updateNetworkTables(matchTime);
    }
    
    private void updateGamePhase(double matchTime) {
        lastPhase = currentPhase;
        
        if (matchTime > 130) { // Last 20 seconds before end
            currentPhase = GamePhase.ENDGAME;
        } else if (matchTime > 110) { // 110-130 seconds
            currentPhase = GamePhase.TELEOP_CYCLE_2;
        } else if (matchTime > 30) { // 30-110 seconds
            currentPhase = GamePhase.TELEOP_CYCLE_1;
        } else if (matchTime > 20) { // 20-30 seconds
            currentPhase = GamePhase.TELEOP_START;
        } else {
            currentPhase = GamePhase.AUTO;
        }
    }
    
    private void updateFieldState(double matchTime) {
        lastFieldState = currentFieldState;
        
        switch (currentPhase) {
            case AUTO:
            case TELEOP_START:
            case ENDGAME:
                currentFieldState = FieldState.BOTH_ACTIVE;
                break;
                
            case TELEOP_CYCLE_1:
            case TELEOP_CYCLE_2:
                // Determine which side is active based on time within cycle
                double timeIntoCycle = getTimeIntoCycle(matchTime);
                if (timeIntoCycle < CYCLE_LENGTH / 2) {
                    currentFieldState = FieldState.RED_ACTIVE;
                } else {
                    currentFieldState = FieldState.BLUE_ACTIVE;
                }
                break;
        }
    }
    
    private void checkForChangeNotification(double matchTime) {
        changeNotificationTriggered = false;
        
        // Only during teleop cycles
        if (currentPhase != GamePhase.TELEOP_CYCLE_1 && currentPhase != GamePhase.TELEOP_CYCLE_2) {
            return;
        }
        
        double timeIntoCycle = getTimeIntoCycle(matchTime);
        double timeUntilChange = Math.abs(CYCLE_LENGTH / 2 - timeIntoCycle);
        
        // Trigger notification when approaching 10 seconds (middle of cycle)
        if (timeUntilChange < 0.5 && !changeNotificationTriggered) {
            changeNotificationTriggered = true;
            notifyChangeHappening(timeUntilChange);
        }
    }
    
    private double getTimeIntoCycle(double matchTime) {
        if (currentPhase == GamePhase.TELEOP_CYCLE_1) {
            return (matchTime - 30) % CYCLE_LENGTH;
        } else if (currentPhase == GamePhase.TELEOP_CYCLE_2) {
            return (matchTime - 70) % CYCLE_LENGTH;
        }
        return 0;
    }
    
    private void notifyListenersIfChanged() {
        if (currentPhase != lastPhase) {
            for (GameStateListener listener : listeners) {
                listener.onPhaseChanged(currentPhase, lastPhase);
            }
        }
        
        if (currentFieldState != lastFieldState) {
            for (GameStateListener listener : listeners) {
                listener.onFieldStateChanged(currentFieldState, lastFieldState);
            }
        }
    }
    
    private void notifyChangeHappening(double timeUntilChange) {
        for (GameStateListener listener : listeners) {
            listener.onChangeNotification(timeUntilChange);
        }
    }
    
    private void updateNetworkTables(double matchTime) {
        table.getEntry("MatchTime").setDouble(matchTime);
        table.getEntry("GamePhase").setString(currentPhase.toString());
        table.getEntry("FieldState").setString(currentFieldState.toString());
        table.getEntry("TimeRemaining").setDouble(matchTime);
        table.getEntry("IsEndgame").setBoolean(currentPhase == GamePhase.ENDGAME);
        table.getEntry("BothSidesActive").setBoolean(currentFieldState == FieldState.BOTH_ACTIVE);
    }
    
    public void addListener(GameStateListener listener) {
        listeners.add(listener);
    }
    
    public void removeListener(GameStateListener listener) {
        listeners.remove(listener);
    }
    
    // Getters
    public GamePhase getCurrentPhase() {
        return currentPhase;
    }
    
    public FieldState getCurrentFieldState() {
        return currentFieldState;
    }
    
    public boolean isBothSidesActive() {
        return currentFieldState == FieldState.BOTH_ACTIVE;
    }
    
    public boolean isRedActive() {
        return currentFieldState == FieldState.RED_ACTIVE || currentFieldState == FieldState.BOTH_ACTIVE;
    }
    
    public boolean isBlueActive() {
        return currentFieldState == FieldState.BLUE_ACTIVE || currentFieldState == FieldState.BOTH_ACTIVE;
    }
    
    public String getPhaseDisplayName() {
        switch (currentPhase) {
            case AUTO:
                return "AUTONOMOUS";
            case TELEOP_START:
                return "TELEOP - Both Active";
            case TELEOP_CYCLE_1:
            case TELEOP_CYCLE_2:
                return "TELEOP - " + currentFieldState.toString();
            case ENDGAME:
                return "ENDGAME";
            default:
                return "UNKNOWN";
        }
    }
}