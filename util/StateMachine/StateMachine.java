package org.firstinspires.ftc.teamcode.centerstage.util.StateMachine;

public class StateMachine<OwnerClass> {
    private State<OwnerClass> currentState;

    public State<OwnerClass> getCurrentState() {
        return currentState;
    }

    OwnerClass owner;

    public StateMachine(OwnerClass owner) {
        this.owner = owner;
    }

    public void changeState(State<OwnerClass> newState) {
        if (currentState != null) {
            currentState.exit();
        }
        currentState = newState;
        currentState.enter(owner);
    }

    public void execute() {
        if (this.currentState == null) return;
        currentState.execute();
    }
}
