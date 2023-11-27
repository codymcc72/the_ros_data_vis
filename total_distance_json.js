// Function to calculate distance between two points
function calculateDistance(point1, point2) {
  const dx = point2.head.position.x - point1.head.position.x;
  const dy = point2.head.position.y - point1.head.position.y;
  return Math.sqrt(dx ** 2 + dy ** 2);
}

// Function to calculate the total distance of the JSON map
function calculateTotalMapDistance() {
  let totalDistance = 0;

  for (let i = 1; i < json.points.length; i++) {
    totalDistance += calculateDistance(json.points[i - 1], json.points[i]);
  }

  return totalDistance;
}

// Example usage
const totalMapDistance = calculateTotalMapDistance();
console.log('Total Map Distance:', totalMapDistance);
