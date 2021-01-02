
const runButton = document.querySelector('#startButton');



const runSim = () => {

const graphWrap =   document.querySelector('#graph-wrap');

while (graphWrap.firstChild) {
    graphWrap.firstChild.remove();
}

let MASS;
MASS = document.getElementById('massInput').value;
if (!MASS) {
   MASS = 0.90;
}
console.log('MASS', MASS);

let DROP_HEIGHT;
DROP_HEIGHT = Number(document.getElementById('dropHeight').value);
if (!DROP_HEIGHT) {
   DROP_HEIGHT = 10;
}
console.log('DROP_HEIGHT', DROP_HEIGHT);

let ROCKET_ON_TIME;
ROCKET_ON_TIME = Number(document.getElementById('rocketOnTime').value);
if (!ROCKET_ON_TIME) {
   ROCKET_ON_TIME = 0.8;
}
console.log('ROCKET_ON_TIME', ROCKET_ON_TIME);

let KP;
KP = Number(document.getElementById('kp').value);
if (!KP) {
   KP = 0.1;
}
console.log('KP', KP);

let KI;
KI = Number(document.getElementById('ki').value);
if (!KI) {
   KI = 0.01;
}
console.log('KI', KI);

let KD;
KD = Number(document.getElementById('kd').value);
if (!KD) {
   KD = 0.05;
}
console.log('KD', KD);

let LENGTH;
LENGTH = Number(document.getElementById('length').value);
if (!LENGTH) {
   LENGTH = 0.75;
}
console.log('LENGTH', LENGTH);

let RADIUS;
RADIUS = Number(document.getElementById('radius').value);
if (!RADIUS) {
   RADIUS = 0.05;
}
console.log('RADIUS', RADIUS);

const CENTER_OF_MASS = 0.3;
const GRAVITY = -9.81;

const secondsToSimulate = 10;
const stepsPerSecond = 1000;
const stepsToSimulate = secondsToSimulate * stepsPerSecond;
const stepDuration = 1/stepsPerSecond;

const getPid = (kp, ki, kd, min, max) => {
  let errorSum = 0;
  let lastError = 0;

  return (input, setPoint) => {
    const error = setPoint - input;
    errorSum += (error * stepDuration);
    const errorDelta = (error - lastError) / stepDuration;
    lastError = error;
    const kiClamped = Math.max(min, Math.min(max, (ki * errorSum)));
    return Math.max(min, Math.min(max, (kp * error) + kiClamped + (kd * errorDelta)));
  }
}

const rocketThrusts = [
[0.002, 1.678],
[0.050, 4.459],
[0.100, 10.431],
[0.200, 24.152],
[0.274, 31.959],
[0.292, 32.702],
[0.310, 27.957],
[0.320, 25.957],
[0.330, 22.857],
[0.340, 19.128],
[0.350, 16.455],
[0.360, 15.314],
[0.380, 13.853],
[0.390, 13.436],
[0.400, 13.271],
[0.450, 12.070],
[0.500, 11.522],
[0.550, 11.266],
[0.600, 10.736],
[0.650, 10.777],
[0.700, 10.276],
[0.800, 10.105],
[0.900, 9.920],
[1.000, 9.693],
[1.310, 9.759],
[1.316, 10.696],
[1.330, 9.628],
[2.380, 9.870],
[2.400, 6.442],
[2.420, 3.674],
[2.440, 0.000]]

const thrusts = [];

for (let i = 0; i < stepsToSimulate; i ++ ) {
  if (i >= ROCKET_ON_TIME * stepsPerSecond) {
    const timeIntoThrust = (i * stepDuration) - ROCKET_ON_TIME;
    const latestThrust = rocketThrusts.find(([t, v]) => t > timeIntoThrust);
    thrusts.push(latestThrust ? latestThrust[1] : 0);
  } else {
    thrusts.push(0);
  }
}

// Moment of Inertia
// I = ((1/4) * Mass * Radius^2) + ((1/12) * Mass * Length^2)

const momentOfInertia = ((1/4) * MASS * Math.pow(RADIUS, 2)) + ((1/12) * MASS * Math.pow(LENGTH, 2))

// Rotation

const thrustAngles = [0];
const rotations = [0];

const thrustXs = [Math.sin(rotations[0] + thrustAngles[0]) * thrusts[0]]
const thrustYs = [Math.cos(rotations[0] + thrustAngles[0]) * thrusts[0]]

const aAccelerations = [0];
const xAccelerations = [thrustXs[0] / MASS];
const yAccelerations = [GRAVITY + (thrustYs[0] / MASS)];

const aVelocities = [0.2];
const yVelocities = [0];
const xVelocities = [0];

const yPositions = [DROP_HEIGHT];
const xPositions = [0];

//const positionPid = getPid(0.1, 0.1, 1.5, -Math.PI/2, Math.PI/2);
const anglePid = getPid(KP, KI, KD, -0.087, 0.087);

for (let i = 1; i < stepsToSimulate; i ++) {
  //const neededAngle = thrusts[i] <= 0 ? 0 : anglePid(xPositions[i-1], 0);
  const nextThrustAngle = thrusts[i] <= 0 ? 0 : anglePid(rotations[i-1], 0);

  thrustAngles.push(nextThrustAngle);
  const absoluteThrustAngle = rotations[i-1] + thrustAngles[i];
  const rocketLateralThrust = Math.sin(thrustAngles[i]) * thrusts[i];

  thrustXs.push(Math.sin(absoluteThrustAngle) * thrusts[i]);
  thrustYs.push(Math.cos(absoluteThrustAngle) * thrusts[i]);

  aAccelerations.push((rocketLateralThrust * CENTER_OF_MASS) / momentOfInertia);
  yAccelerations.push(GRAVITY + ((thrustYs[i]) / MASS));
  xAccelerations.push((thrustXs[i] / MASS));

  aVelocities.push(aVelocities[i-1] + (aAccelerations[i] * stepDuration));
  yVelocities.push(yVelocities[i-1] + (yAccelerations[i] * stepDuration));
  xVelocities.push(xVelocities[i-1] + (xAccelerations[i] * stepDuration));

  rotations.push(rotations[i-1] + (aVelocities[i] * stepDuration));
  yPositions.push(yPositions[i-1] + (yVelocities[i] * stepDuration) > 0 ? yPositions[i-1] + (yVelocities[i] * stepDuration) : 0);
  xPositions.push(xPositions[i-1] + (xVelocities[i] * stepDuration));
}



new uPlot(
  {
    title: "Angles",
    id: "chart1",
    class: "my-chart",
    width: window.innerWidth,
    height: window.innerHeight / 2,
    series: [
      {
        value: (self, rawValue) => `${rawValue / stepsPerSecond}s`
      },
      {
        label: "Thruster Angle",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Rocket Rotation",
        stroke: "rgba(0, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
    ],
  },
  [
    thrusts.map((x, i) => i),
    thrustAngles,
    rotations
  ],
  document.querySelector('#graph-wrap')
);

new uPlot(
  {
    title: "Rocket Y",
    id: "chart1",
    class: "my-chart",
    width: window.innerWidth,
    height: window.innerHeight / 2,
    series: [
      {
        value: (self, rawValue) => `${rawValue / stepsPerSecond}s`
      },
      {
        label: "Thrust Y",
        stroke: "rgba(255, 0, 0, 1)",
        dash: [10, 5],
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Acceleration",
        stroke: "rgba(0, 160, 160, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Velocity",
        stroke: "rgba(0, 0, 255, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Position",
        stroke: "rgba(0, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
    ],
  },
  [
    thrusts.map((x, i) => i),
    thrustYs,
    yAccelerations,
    yVelocities,
    yPositions
  ],
  document.querySelector('#graph-wrap')
);

new uPlot(
  {
    title: "Rocket X",
    id: "chart1",
    class: "my-chart",
    width: window.innerWidth,
    height: window.innerHeight / 2,
    series: [
      {
        value: (self, rawValue) => `${rawValue / stepsPerSecond}s`
      },
      {
        label: "Thrust X",
        stroke: "rgba(255, 0, 0, 1)",
        dash: [10, 5],
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Acceleration",
        stroke: "rgba(0, 160, 160, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Velocity",
        stroke: "rgba(0, 0, 255, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Position",
        stroke: "rgba(0, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
    ],
  },
  [
    thrusts.map((x, i) => i),
    thrustXs,
    xAccelerations,
    xVelocities,
    xPositions
  ],
  document.querySelector('#graph-wrap')
);

animate(thrusts, yPositions, xPositions, thrustAngles, rotations, CENTER_OF_MASS, stepsPerSecond, LENGTH, RADIUS * 2)


}

runButton.addEventListener('click', () => runSim());
