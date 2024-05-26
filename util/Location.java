package org.firstinspires.ftc.teamcode.centerstage.util;


import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;

public class Location {

    public double x, y, angle = 0;

    public Location(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Location(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public Location(Location location) {
        this.x = location.x;
        this.y = location.y;
        this.angle = location.angle;
    }

    public Location(Location location, double xOffset, double yOffset, double angleOffset) {
        this.x = location.x + xOffset;
        this.y = location.y + yOffset;
        this.angle = location.angle + angleOffset;
    }

    public void flipX() {
        this.x *= -1;
    }

    public Location flipY() {
        this.y *= -1;
        return this;
    }

    public void flipAngle() {
        this.angle *= -1;
    }

    public Location offsetY(double offset) {
        return new Location(this.x, this.y + offset, this.angle);
    }

    public Location add(Location location) {
        Location result = this;
        result = result.addX(location.x);
        result = result.addY(location.y);
        return result;
    }

    public Location addX(double x) {
        return new Location(this.x + x, this.y, this.angle);
    }

    public Location addY(double y) {
        return new Location(this.x, this.y + y, this.angle);
    }

    public Location subtract(Location location) {
        Location result = this;
        result = result.subtractX(location.x);
        result = result.subtractY(location.y);
        return result;
    }

    public Location subtractX(double x) {
        return this.addX(-x);
    }


    public Location subtractY(double y) {
        return this.addY(-y);
    }

    public Location offsetX(double offset) {
        return new Location(this.x + offset, this.y, this.angle);
    }

    public Location stayOnX(DriveClass drive) {
        return new Location(drive.getPosX(), this.y);
    }

    public Location stayOnY(DriveClass drive) {
        return new Location(this.x, drive.getPosY());
    }

}
