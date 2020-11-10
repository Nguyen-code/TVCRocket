const canvas = document.getElementById('visualizer-canvas');
const ctx = canvas.getContext('2d');

canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

const width = canvas.width;
const height = canvas.height;

const groundHeightPx = 80;
const exhaustMaxHeight = 0.6;
const exhaustMaxWidth = 0.15;

ctx.scale(1, -1);
ctx.translate(0, -1 * height);

const animate = (
  thrusts,
  yPositions,
  xPositions,
  thrustAngles,
  rotations,
  centerOfGravityOffset,
  stepsPerSecond,
  rocketHeight,
  rocketWidth,
) => {
  const maxPos = Math.max(...yPositions) ;
  const maxThrust = Math.max(...thrusts);
  const pxPerMeter = (height - (groundHeightPx + 40)) / maxPos;

  const startTime = Date.now();

  const metersToPx = (meters) => meters * pxPerMeter;
  const metersToYposition = (meters) => metersToPx(meters + (groundHeightPx / pxPerMeter));

  const rocketWidthPx = metersToPx(rocketWidth);
  const rocketHeightPx = metersToPx(rocketHeight);

  const frame = () => {
    const index = Math.floor(((Date.now() - startTime) / 1000) * stepsPerSecond);
    if (index > yPositions.length) {
      return;
    }

    ctx.clearRect(0, 0, width, height)
    const thrust = thrusts[index];
    const yPosition = Math.max(0, yPositions[index]);

    const centerX = metersToPx(xPositions[index] || 0) + (width / 2);
    const bottomY = metersToYposition(yPosition);
    const centerOfGravityY = bottomY + metersToPx(centerOfGravityOffset);

    ctx.save();

    ctx.translate(centerX, centerOfGravityY);
    ctx.rotate(-rotations[index]);
    ctx.translate(-centerX, -centerOfGravityY);

    ctx.fillStyle = '#000';
    ctx.fillRect(
      centerX  - (rocketWidthPx / 2),
      bottomY,
      rocketWidthPx,
      rocketHeightPx
    );

    ctx.translate(centerX, centerOfGravityY);
    ctx.rotate(-thrustAngles[index]);
    ctx.translate(-centerX, -centerOfGravityY);

    const exhaustHeight = metersToPx((thrust / maxThrust) * exhaustMaxHeight);
    const exhaustWidth = metersToPx((thrust / maxThrust) + Math.random() * (thrust / maxThrust)) * exhaustMaxWidth;
    ctx.fillStyle = `rgb(255,${Math.random() * 150 + 75},${Math.random() * 100 + 75})`;
    ctx.fillRect(
      centerX - (exhaustWidth / 2),
      bottomY - exhaustHeight,
      exhaustWidth,
      exhaustHeight
    );
    ctx.restore();

    ctx.fillRect(width / 2, 0, 1, height);

    ctx.fillStyle = "#00aa00";
    ctx.fillRect(
      0,
      0,
      width,
      groundHeightPx
    )

    for (let i = 0; i < 200; i += 10) {
      const y = metersToYposition(i);
      ctx.fillStyle = "rgba(0,0,0,0.25)";
      ctx.fillRect(0, Math.round(y), width, 1);
      ctx.save();
      ctx.translate(0, y)
      ctx.scale(1, -1);
      ctx.translate(0, -y)
      ctx.fillStyle = "rgba(0,0,0,0.75)";
      ctx.fillText(`${i}m`, 10, y - 5)
      ctx.restore();
    }

    requestAnimationFrame(frame);
  }

  frame();
}
