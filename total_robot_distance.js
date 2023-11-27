// Variable to store the total distance traveled by the robot
let totalRobotDistance = 0;

// Function to update the total distance when a new position is received
function updateTotalRobotDistance(newLatitude, newLongitude) {
  if (homeLatitude !== undefined && homeLongitude !== undefined) {
    const dx = newLongitude - homeLongitude;
    const dy = newLatitude - homeLatitude;
    const distance = Math.sqrt(dx ** 2 + dy ** 2);

    // Add the distance to the total
    totalRobotDistance += distance;

    // Update home coordinates for the next calculation
    homeLatitude = newLatitude;
    homeLongitude = newLongitude;
  }
}

// Example usage (call this function when a new position is received)
const newLatitude = /* get the new latitude from the robot */;
const newLongitude = /* get the new longitude from the robot */;
updateTotalRobotDistance(newLatitude, newLongitude);

// Example usage
console.log('Total Robot Distance:', totalRobotDistance);
