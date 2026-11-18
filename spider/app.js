(() => {
  var __defProp = Object.defineProperty;
  var __defNormalProp = (obj, key, value) => key in obj ? __defProp(obj, key, { enumerable: true, configurable: true, writable: true, value }) : obj[key] = value;
  var __publicField = (obj, key, value) => __defNormalProp(obj, typeof key !== "symbol" ? key + "" : key, value);

  // src/core/math.ts
  var vec = (x = 0, y = 0) => ({ x, y });
  var dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  function rotate(a, angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
  }
  var clamp = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;
  var lerp = (a, b, t) => a + (b - a) * t;
  var easeInOutCubic = (t) => t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
  var easeOutCubic = (t) => 1 - Math.pow(1 - clamp(t, 0, 1), 3);
  function wrapAngle(a) {
    let x = a;
    while (x <= -Math.PI) x += Math.PI * 2;
    while (x > Math.PI) x -= Math.PI * 2;
    return x;
  }
  function dampAngle(current, target, halfLife, dt) {
    const delta = wrapAngle(target - current);
    return current + delta * (1 - Math.pow(2, -dt / Math.max(halfLife, 1e-6)));
  }
  function smoothDamp(current, target, velocity, smoothTime, dt, maxSpeed = Infinity) {
    const st = Math.max(1e-4, smoothTime);
    const omega = 2 / st;
    const x = omega * dt;
    const exp = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);
    let change = current - target;
    const maxChange = maxSpeed * st;
    change = clamp(change, -maxChange, maxChange);
    const t = current - change;
    const temp = (velocity + omega * change) * dt;
    let newVel = (velocity - omega * temp) * exp;
    let out = t + (change + temp) * exp;
    if (target - current > 0 === out > target) {
      out = target;
      newVel = (out - target) / dt;
    }
    return [out, newVel];
  }

  // src/spider/ik.ts
  function solveTwoBone(hip, target, a, b, awayFrom) {
    const dx = target.x - hip.x;
    const dy = target.y - hip.y;
    const raw = Math.hypot(dx, dy);
    const dirX = raw > 1e-6 ? dx / raw : 1;
    const dirY = raw > 1e-6 ? dy / raw : 0;
    const d = Math.min(Math.max(raw, Math.abs(a - b) + 1e-3), a + b - 1e-3);
    const cosAngle = (a * a + d * d - b * b) / (2 * a * d);
    const angle = Math.acos(Math.min(1, Math.max(-1, cosAngle)));
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const k1 = { x: hip.x + (dirX * c - dirY * s) * a, y: hip.y + (dirX * s + dirY * c) * a };
    const k2 = { x: hip.x + (dirX * c + dirY * s) * a, y: hip.y + (-dirX * s + dirY * c) * a };
    const d1 = (k1.x - awayFrom.x) ** 2 + (k1.y - awayFrom.y) ** 2;
    const d2 = (k2.x - awayFrom.x) ** 2 + (k2.y - awayFrom.y) ** 2;
    return {
      knee: d1 >= d2 ? k1 : k2,
      end: { x: hip.x + dirX * d, y: hip.y + dirY * d },
      extension: d / (a + b)
    };
  }

  // src/spider/spider.ts
  var BASE_LEGS = [
    { hip: vec(9.5, 3.6), rest: vec(50, 32), a: 30, b: 34 },
    { hip: vec(6.5, 5.6), rest: vec(20, 52), a: 30, b: 36 },
    { hip: vec(2.5, 6), rest: vec(-14, 50), a: 27, b: 32 },
    { hip: vec(-1, 4.6), rest: vec(-42, 35), a: 30, b: 37 }
  ];
  var BODY_Z = 9;
  var Spider = class {
    constructor(pos, skin, heading = 0) {
      __publicField(this, "pos");
      __publicField(this, "vel", vec());
      __publicField(this, "heading");
      __publicField(this, "angVel", 0);
      __publicField(this, "z", BODY_Z);
      __publicField(this, "mode", "walk");
      __publicField(this, "legs", []);
      __publicField(this, "skin");
      /** Body-center goal; the director moves it, the spider chases it. */
      __publicField(this, "goal");
      /** Desired facing when standing still (null = keep current). */
      __publicField(this, "face", null);
      __publicField(this, "speedLimit", 420);
      /** Abdomen lag when turning (secondary motion). */
      __publicField(this, "abdomenSwing", 0);
      __publicField(this, "abdomenSwingVel", 0);
      /** 0..1, palps tap and the body leans in. */
      __publicField(this, "tasting", 0);
      /** 0..1, used for the idle "looking around" palps. */
      __publicField(this, "curiosity", 0);
      __publicField(this, "time", 0);
      __publicField(this, "steps", 0);
      /** Distance walked, in world units (the silk counter uses it). */
      __publicField(this, "traveled", 0);
      __publicField(this, "bobPhase", 0);
      __publicField(this, "jump", null);
      __publicField(this, "rappel", null);
      __publicField(this, "onStep");
      __publicField(this, "onLand");
      this.pos = { ...pos };
      this.goal = { ...pos };
      this.heading = heading;
      this.skin = skin;
      this.rig();
    }
    get scale() {
      return this.skin.scale;
    }
    /** Build legs from the skin (sizes) and plant them around the body. */
    rig() {
      const s = this.skin.scale;
      const L = this.skin.legLength;
      this.legs = [];
      for (const side of [-1, 1]) {
        BASE_LEGS.forEach((base, pair) => {
          const hip = vec(base.hip.x * s, base.hip.y * side * s);
          const restRel = vec((base.rest.x - base.hip.x) * L, (base.rest.y - base.hip.y) * L);
          const rest = vec(hip.x + restRel.x * s, hip.y + restRel.y * side * s);
          this.legs.push({
            side,
            pair,
            group: (pair + (side === 1 ? 1 : 0)) % 2,
            hip,
            rest,
            a: base.a * s * L,
            b: base.b * s * L,
            foot: vec(),
            footZ: 0,
            stepping: false,
            stepT: 0,
            stepDur: 0.15,
            stepFrom: vec(),
            stepTo: vec(),
            hipW: vec(),
            kneeW: vec(),
            endW: vec(),
            kneeZ: 0,
            jitter: (pair * 7 + (side + 1) * 3) % 5 / 5
          });
        });
      }
      this.plantAll();
      this.solveLegs();
    }
    setSkin(skin) {
      this.skin = skin;
      this.rig();
    }
    plantAll(at2 = this.pos) {
      for (const leg of this.legs) {
        leg.foot = this.restWorld(leg, at2);
        leg.stepping = false;
        leg.footZ = 0;
      }
    }
    restWorld(leg, at2 = this.pos, heading = this.heading, shrink = 1) {
      const r = rotate({ x: leg.rest.x * shrink, y: leg.rest.y * shrink }, heading);
      return { x: at2.x + r.x, y: at2.y + r.y };
    }
    /** Leg reach used for step thresholds. */
    get reach() {
      return 48 * this.skin.scale * Math.sqrt(this.skin.legLength);
    }
    get headPos() {
      const r = rotate({ x: 15 * this.scale, y: 0 }, this.heading);
      return { x: this.pos.x + r.x, y: this.pos.y + r.y };
    }
    get spinnerets() {
      const back = (14 + 15 * this.skin.abdomen.length) * this.scale;
      const r = rotate({ x: -back, y: 0 }, this.heading + this.abdomenSwing);
      return { x: this.pos.x + r.x, y: this.pos.y + r.y };
    }
    get speed() {
      return Math.hypot(this.vel.x, this.vel.y);
    }
    get busy() {
      return this.mode !== "walk";
    }
    /** Teleport (no animation). */
    place(p, heading = this.heading) {
      this.pos = { ...p };
      this.goal = { ...p };
      this.vel = vec();
      this.heading = heading;
      this.mode = "walk";
      this.jump = null;
      this.rappel = null;
      this.plantAll();
      this.solveLegs();
    }
    jumpTo(to, height) {
      const d = dist(this.pos, to);
      this.mode = "jump";
      this.jump = {
        from: { ...this.pos },
        to: { ...to },
        height: height ?? clamp(30 + d * 0.22, 30, 140),
        t: 0,
        crouch: 0.12,
        flight: clamp(0.28 + d / 1500, 0.3, 0.75),
        land: 0.16,
        landed: false
      };
    }
    /** Drop down on a thread from `anchor` (above) to `to`. */
    rappelTo(anchor, to, duration) {
      this.mode = "rappel";
      const from = { x: to.x, y: anchor.y + 30 };
      this.place(from, Math.PI / 2);
      this.mode = "rappel";
      this.rappel = { anchor: { ...anchor }, from, to: { ...to }, t: 0, dur: duration ?? clamp(0.6 + (to.y - from.y) / 1400, 0.7, 2.2) };
    }
    update(dt) {
      if (dt <= 0) return;
      this.time += dt;
      const prevHeading = this.heading;
      if (this.mode === "walk") this.updateWalk(dt);
      else if (this.mode === "jump") this.updateJump(dt);
      else this.updateRappel(dt);
      this.angVel = wrapAngle(this.heading - prevHeading) / dt;
      const target = clamp(-this.angVel * 0.045, -0.5, 0.5);
      const k = 70;
      const c = 2 * Math.sqrt(k) * 0.55;
      this.abdomenSwingVel += (k * (target - this.abdomenSwing) - c * this.abdomenSwingVel) * dt;
      this.abdomenSwing += this.abdomenSwingVel * dt;
      this.tasting = Math.max(0, this.tasting - dt * 0.9);
      this.solveLegs();
    }
    updateWalk(dt) {
      const [x, vx] = smoothDamp(this.pos.x, this.goal.x, this.vel.x, 0.1, dt, this.speedLimit);
      const [y, vy] = smoothDamp(this.pos.y, this.goal.y, this.vel.y, 0.1, dt, this.speedLimit);
      const moved = Math.hypot(x - this.pos.x, y - this.pos.y);
      this.pos = { x, y };
      this.vel = { x: vx, y: vy };
      this.traveled += moved;
      const speed = this.speed;
      if (speed > 14) {
        const desired = Math.atan2(vy, vx);
        const halfLife = 0.05 + 0.09 * (1 - Math.min(speed / 260, 1));
        this.heading = dampAngle(this.heading, desired, halfLife, dt);
      } else if (this.face !== null) {
        this.heading = dampAngle(this.heading, this.face, 0.14, dt);
      }
      this.bobPhase += moved / (26 * this.scale);
      const bob = speed > 8 ? Math.sin(this.bobPhase * Math.PI * 2) * 0.7 : Math.sin(this.time * 2.2) * 0.25;
      this.z = BODY_Z * this.scale - this.tasting * 2.5 + bob;
      this.updateGait(dt, speed);
    }
    updateGait(dt, speed) {
      const s = this.scale;
      const stepDur = clamp(0.2 - speed * 16e-5, 0.07, 0.2);
      const lift = clamp(5 + speed * 0.014, 5, 13) * s;
      const moving = speed > 10;
      const threshold = this.reach * (moving ? clamp(0.3 + speed / 900, 0.3, 0.7) : 0.16);
      const leadLen = Math.min(speed * stepDur * 1.1, threshold * 0.85);
      const lead = speed > 1e-3 ? { x: this.vel.x / speed * leadLen, y: this.vel.y / speed * leadLen } : { x: 0, y: 0 };
      const steppingIn = [0, 0];
      for (const leg of this.legs) if (leg.stepping) steppingIn[leg.group]++;
      const homes = this.legs.map((leg) => this.restWorld(leg));
      const err = [0, 0];
      this.legs.forEach((leg, i) => {
        if (!leg.stepping) err[leg.group] = Math.max(err[leg.group], dist(leg.foot, homes[i]));
      });
      this.legs.forEach((leg, i) => {
        const home = homes[i];
        const want = { x: home.x + lead.x, y: home.y + lead.y };
        if (leg.stepping) {
          leg.stepT += dt / leg.stepDur;
          leg.stepTo = { x: lerp(leg.stepTo.x, want.x, 0.3), y: lerp(leg.stepTo.y, want.y, 0.3) };
          if (leg.stepT < 0) return;
          const t = Math.min(1, leg.stepT);
          const e = easeInOutCubic(t);
          leg.foot = { x: lerp(leg.stepFrom.x, leg.stepTo.x, e), y: lerp(leg.stepFrom.y, leg.stepTo.y, e) };
          leg.footZ = Math.sin(Math.PI * t) * lift;
          if (t >= 1) {
            leg.stepping = false;
            leg.footZ = 0;
            this.steps++;
            this.onStep?.(leg);
          }
          return;
        }
        const d = dist(leg.foot, home);
        const other = 1 - leg.group;
        const groupTurn = steppingIn[other] === 0 && err[leg.group] >= threshold && err[leg.group] >= err[other] * 0.8;
        const emergency = d > Math.max(threshold * 1.6, this.reach * 0.75);
        const settle = !moving && steppingIn[other] === 0 && d > 3.5 * s && err[leg.group] >= err[other];
        if (groupTurn && d > threshold * 0.5 || emergency || settle) {
          leg.stepping = true;
          leg.stepT = -leg.jitter * 0.15;
          leg.stepDur = stepDur * (0.92 + leg.jitter * 0.16);
          leg.stepFrom = { ...leg.foot };
          leg.stepTo = want;
          steppingIn[leg.group]++;
        }
      });
    }
    updateJump(dt) {
      const j = this.jump;
      j.t += dt;
      const base = BODY_Z * this.scale;
      if (j.t < j.crouch) {
        this.z = base - 4 * this.scale * easeOutCubic(j.t / j.crouch);
        this.vel = vec();
        this.heading = dampAngle(this.heading, Math.atan2(j.to.y - j.from.y, j.to.x - j.from.x), 0.04, dt);
        return;
      }
      const ft = j.t - j.crouch;
      if (ft < j.flight) {
        const u = ft / j.flight;
        const e = easeInOutCubic(u);
        const prev = this.pos;
        this.pos = { x: lerp(j.from.x, j.to.x, e), y: lerp(j.from.y, j.to.y, e) };
        this.vel = { x: (this.pos.x - prev.x) / dt, y: (this.pos.y - prev.y) / dt };
        this.traveled += dist(prev, this.pos);
        this.z = base + j.height * 4 * u * (1 - u);
        this.heading = dampAngle(this.heading, Math.atan2(j.to.y - j.from.y, j.to.x - j.from.x), 0.05, dt);
        const tuck = 0.55 + 0.35 * Math.max(0, u - 0.6) / 0.4;
        for (const leg of this.legs) {
          leg.stepping = false;
          leg.foot = this.restWorld(leg, this.pos, this.heading, tuck);
          leg.footZ = Math.max(0, this.z - 6 * this.scale);
        }
        return;
      }
      if (!j.landed) {
        j.landed = true;
        this.pos = { ...j.to };
        this.goal = { ...j.to };
        this.vel = vec();
        this.plantAll();
        this.onLand?.();
      }
      const lt = (ft - j.flight) / j.land;
      this.z = base - 3.5 * this.scale * Math.sin(Math.PI * Math.min(1, lt));
      if (lt >= 1) {
        this.mode = "walk";
        this.jump = null;
        this.z = base;
      }
    }
    updateRappel(dt) {
      const r = this.rappel;
      r.t += dt;
      const u = Math.min(1, r.t / r.dur);
      const e = easeOutCubic(u);
      const prev = this.pos;
      const sway = Math.sin(this.time * 2.6) * 7 * (1 - e);
      this.pos = { x: r.to.x + sway, y: lerp(r.from.y, r.to.y, e) };
      this.vel = { x: (this.pos.x - prev.x) / dt, y: (this.pos.y - prev.y) / dt };
      this.traveled += dist(prev, this.pos);
      this.heading = Math.PI / 2 + Math.sin(this.time * 1.7) * 0.12 * (1 - e);
      this.z = BODY_Z * this.scale + 70 * (1 - e) * this.scale;
      this.legs.forEach((leg, i) => {
        leg.stepping = false;
        const paddle = 0.68 + 0.08 * Math.sin(this.time * 7 + i * 1.3);
        leg.foot = this.restWorld(leg, this.pos, this.heading, paddle);
        leg.footZ = Math.max(0, this.z - 5 * this.scale);
      });
      if (u >= 1) {
        this.rappel = null;
        this.mode = "walk";
        this.pos = { ...r.to };
        this.goal = { ...r.to };
        this.vel = vec();
        this.z = BODY_Z * this.scale;
        this.plantAll();
        this.onLand?.();
      }
    }
    /** Solve IK for drawing. */
    solveLegs() {
      const center = this.pos;
      for (const leg of this.legs) {
        const h = rotate(leg.hip, this.heading);
        leg.hipW = { x: center.x + h.x, y: center.y + h.y };
        const res = solveTwoBone(leg.hipW, leg.foot, leg.a, leg.b, center);
        leg.kneeW = res.knee;
        leg.endW = res.end;
        leg.kneeZ = this.z + (10 + 12 * (1 - res.extension)) * this.scale + leg.footZ * 0.4;
      }
    }
  };

  // src/spider/draw.ts
  var DEFAULT_DRAW = { shadow: 1, light: { x: 0.42, y: 0.78 }, px: 1 };
  var SHADOW_RGB = "20, 18, 46";
  function at(sp, x, y) {
    const r = rotate({ x, y }, sp.heading);
    return { x: sp.pos.x + r.x, y: sp.pos.y + r.y };
  }
  function bodyGeometry(sp) {
    const s = sp.scale;
    const skin = sp.skin;
    const pedicel = at(sp, -3.5 * s, 0);
    const abdR = { x: 15 * skin.abdomen.length * s, y: 11.5 * skin.abdomen.width * s };
    const abdAngle = sp.heading + sp.abdomenSwing;
    const off = rotate({ x: -abdR.x * 0.9, y: 0 }, abdAngle);
    return {
      ceph: at(sp, 6 * s, 0),
      cephR: { x: 10.5 * s, y: 8.6 * s },
      abd: { x: pedicel.x + off.x, y: pedicel.y + off.y },
      abdR,
      abdAngle,
      pedicel
    };
  }
  function shapePath(ctx2, shape, c, rx, ry, angle) {
    ctx2.save();
    ctx2.translate(c.x, c.y);
    ctx2.rotate(angle);
    ctx2.beginPath();
    switch (shape) {
      case "ellipse":
        ctx2.ellipse(0, 0, rx, ry, 0, 0, Math.PI * 2);
        break;
      case "circle":
        ctx2.arc(0, 0, (rx + ry) / 2, 0, Math.PI * 2);
        break;
      case "rect":
        ctx2.roundRect(-rx, -ry, rx * 2, ry * 2, Math.min(rx, ry) * 0.28);
        break;
      case "hex":
        for (let k = 0; k < 6; k++) {
          const a = k / 6 * Math.PI * 2;
          const x = Math.cos(a) * rx;
          const y = Math.sin(a) * ry * 1.12;
          if (k === 0) ctx2.moveTo(x, y);
          else ctx2.lineTo(x, y);
        }
        ctx2.closePath();
        break;
      case "diamond":
        ctx2.moveTo(rx * 1.08, 0);
        ctx2.lineTo(0, ry * 1.08);
        ctx2.lineTo(-rx * 1.08, 0);
        ctx2.lineTo(0, -ry * 1.08);
        ctx2.closePath();
        break;
    }
    ctx2.restore();
  }
  function jointPath(ctx2, shape, p, r, angle) {
    ctx2.beginPath();
    if (shape === "circle") {
      ctx2.arc(p.x, p.y, r, 0, Math.PI * 2);
      return;
    }
    const a0 = shape === "diamond" ? angle + Math.PI / 4 : angle;
    const d = shape === "diamond" ? r * 1.35 : r * 1.25;
    for (let k = 0; k < 4; k++) {
      const a = a0 + Math.PI / 4 + k * Math.PI / 2;
      const x = p.x + Math.cos(a) * d;
      const y = p.y + Math.sin(a) * d;
      if (k === 0) ctx2.moveTo(x, y);
      else ctx2.lineTo(x, y);
    }
    ctx2.closePath();
  }
  function stroke(ctx2, skin, color, width, s, opts) {
    if (skin.halo) {
      ctx2.strokeStyle = "rgba(255, 255, 255, 0.92)";
      ctx2.lineWidth = width + 3.2 * s;
      ctx2.stroke();
    }
    if (skin.glow > 0) {
      ctx2.shadowColor = color;
      ctx2.shadowBlur = 10 * skin.glow * opts.px * s;
    }
    ctx2.strokeStyle = color;
    ctx2.lineWidth = width;
    ctx2.stroke();
    if (skin.glow > 0) {
      ctx2.shadowBlur = 0;
      ctx2.shadowColor = "transparent";
    }
  }
  function drawSpider(ctx2, sp, opts = DEFAULT_DRAW) {
    const skin = sp.skin;
    const s = sp.scale;
    const geo = bodyGeometry(sp);
    const lw = 1.7 * s * skin.legWidth;
    ctx2.save();
    ctx2.lineCap = "round";
    ctx2.lineJoin = "round";
    if (opts.shadow > 0) drawShadow(ctx2, sp, geo, opts, lw);
    ctx2.strokeStyle = skin.colors.ink;
    ctx2.globalAlpha = 0.45;
    ctx2.lineWidth = 0.9 * s;
    ctx2.beginPath();
    const c = 2.8 * s;
    for (const leg of sp.legs) {
      const r = sp.restWorld(leg);
      ctx2.moveTo(r.x - c, r.y - c);
      ctx2.lineTo(r.x + c, r.y + c);
      ctx2.moveTo(r.x + c, r.y - c);
      ctx2.lineTo(r.x - c, r.y + c);
    }
    ctx2.stroke();
    ctx2.globalAlpha = 1;
    ctx2.setLineDash([3 * s, 3 * s]);
    ctx2.lineWidth = 1.1 * s;
    for (const leg of sp.legs) {
      if (!leg.stepping) continue;
      const mx = (leg.stepFrom.x + leg.stepTo.x) / 2;
      const my = (leg.stepFrom.y + leg.stepTo.y) / 2;
      const ox = mx - sp.pos.x;
      const oy = my - sp.pos.y;
      const ol = Math.hypot(ox, oy) || 1;
      const bulge = Math.hypot(leg.stepTo.x - leg.stepFrom.x, leg.stepTo.y - leg.stepFrom.y) * 0.35;
      ctx2.strokeStyle = skin.colors.legs[leg.group];
      ctx2.beginPath();
      ctx2.moveTo(leg.stepFrom.x, leg.stepFrom.y);
      ctx2.quadraticCurveTo(mx + ox / ol * bulge, my + oy / ol * bulge, leg.stepTo.x, leg.stepTo.y);
      ctx2.stroke();
    }
    ctx2.setLineDash([]);
    for (const leg of sp.legs) drawLeg(ctx2, sp, leg, lw, opts);
    ctx2.fillStyle = skin.colors.fill;
    shapePath(ctx2, skin.body, geo.abd, geo.abdR.x, geo.abdR.y, geo.abdAngle);
    ctx2.fill();
    stroke(ctx2, skin, skin.colors.ink, 1.4 * s, s, opts);
    ctx2.fillStyle = skin.colors.fill;
    shapePath(ctx2, skin.body, geo.ceph, geo.cephR.x, geo.cephR.y, sp.heading);
    ctx2.fill();
    stroke(ctx2, skin, skin.colors.ink, 1.4 * s, s, opts);
    ctx2.save();
    ctx2.globalAlpha = 0.55;
    ctx2.strokeStyle = skin.colors.ink;
    ctx2.lineWidth = 0.8 * s;
    ctx2.setLineDash([2.5 * s, 2.5 * s]);
    ctx2.beginPath();
    const tail = rotate({ x: -geo.abdR.x * 1.15, y: 0 }, geo.abdAngle);
    ctx2.moveTo(geo.abd.x + tail.x, geo.abd.y + tail.y);
    ctx2.lineTo(geo.pedicel.x, geo.pedicel.y);
    const nose = at(sp, 24 * s, 0);
    ctx2.lineTo(nose.x, nose.y);
    ctx2.stroke();
    ctx2.setLineDash([]);
    shapePath(ctx2, skin.body, geo.abd, geo.abdR.x * 0.55, geo.abdR.y * 0.55, geo.abdAngle);
    ctx2.stroke();
    ctx2.restore();
    jointPath(ctx2, skin.joints, geo.pedicel, 1.8 * s, sp.heading);
    ctx2.fillStyle = "#ffffff";
    ctx2.fill();
    ctx2.strokeStyle = skin.colors.ink;
    ctx2.lineWidth = 1.2 * s;
    ctx2.stroke();
    drawEyes(ctx2, sp, geo, opts);
    const v = { x: sp.vel.x * 0.11, y: sp.vel.y * 0.11 };
    const vl = Math.hypot(v.x, v.y);
    if (vl > 4 * s) {
      const tip = { x: sp.pos.x + v.x, y: sp.pos.y + v.y };
      const ux = v.x / vl;
      const uy = v.y / vl;
      ctx2.beginPath();
      ctx2.moveTo(sp.pos.x, sp.pos.y);
      ctx2.lineTo(tip.x, tip.y);
      ctx2.moveTo(tip.x - ux * 4 * s - uy * 2.6 * s, tip.y - uy * 4 * s + ux * 2.6 * s);
      ctx2.lineTo(tip.x, tip.y);
      ctx2.lineTo(tip.x - ux * 4 * s + uy * 2.6 * s, tip.y - uy * 4 * s - ux * 2.6 * s);
      stroke(ctx2, skin, skin.colors.accent, 1.2 * s, s, opts);
    }
    ctx2.restore();
  }
  function drawLeg(ctx2, sp, leg, lw, opts) {
    const skin = sp.skin;
    const s = sp.scale;
    const color = skin.colors.legs[leg.group];
    ctx2.beginPath();
    ctx2.moveTo(leg.hipW.x, leg.hipW.y);
    ctx2.lineTo(leg.kneeW.x, leg.kneeW.y);
    ctx2.lineTo(leg.endW.x, leg.endW.y);
    stroke(ctx2, skin, color, lw, s, opts);
    const femur = Math.atan2(leg.kneeW.y - leg.hipW.y, leg.kneeW.x - leg.hipW.x);
    for (const p of [leg.hipW, leg.kneeW]) {
      jointPath(ctx2, skin.joints, p, 2.2 * s, femur);
      ctx2.fillStyle = skin.colors.eye;
      ctx2.fill();
      ctx2.strokeStyle = color;
      ctx2.lineWidth = 1.2 * s;
      ctx2.stroke();
    }
    jointPath(ctx2, skin.joints, leg.endW, (leg.stepping ? 2.8 : 2) * s, femur);
    if (leg.stepping) {
      ctx2.fillStyle = "#ffffff";
      ctx2.fill();
      ctx2.strokeStyle = color;
      ctx2.lineWidth = 1.2 * s;
      ctx2.stroke();
    } else {
      ctx2.fillStyle = skin.colors.ink;
      ctx2.fill();
    }
  }
  function drawShadow(ctx2, sp, geo, opts, lw) {
    const skin = sp.skin;
    const off = { x: opts.light.x * sp.z, y: opts.light.y * sp.z };
    const a = Math.max(0.35, 1 - (sp.z - 9 * sp.scale) / 160);
    ctx2.strokeStyle = `rgba(${SHADOW_RGB}, ${0.13 * opts.shadow * a})`;
    ctx2.lineWidth = lw * 2.2;
    ctx2.beginPath();
    for (const leg of sp.legs) {
      const k = leg.footZ / Math.max(1, sp.z);
      ctx2.moveTo(leg.hipW.x + off.x, leg.hipW.y + off.y);
      ctx2.lineTo(leg.kneeW.x + off.x * 1.6, leg.kneeW.y + off.y * 1.6);
      ctx2.lineTo(leg.endW.x + off.x * k, leg.endW.y + off.y * k);
    }
    ctx2.stroke();
    ctx2.fillStyle = `rgba(${SHADOW_RGB}, ${0.1 * opts.shadow * a})`;
    shapePath(ctx2, skin.body, { x: geo.abd.x + off.x, y: geo.abd.y + off.y }, geo.abdR.x, geo.abdR.y, geo.abdAngle);
    ctx2.fill();
    shapePath(ctx2, skin.body, { x: geo.ceph.x + off.x, y: geo.ceph.y + off.y }, geo.cephR.x, geo.cephR.y, sp.heading);
    ctx2.fill();
  }
  function drawEyes(ctx2, sp, geo, opts) {
    const skin = sp.skin;
    const s = sp.scale;
    const dot = (p, r, color) => {
      ctx2.fillStyle = color;
      ctx2.beginPath();
      ctx2.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx2.fill();
    };
    const glint = (p, r) => dot({ x: p.x - r * 0.33, y: p.y - r * 0.38 }, Math.max(0.4, r * 0.34), "#ffffff");
    const glow = (on, blur) => {
      ctx2.shadowColor = on ? skin.colors.accent : "transparent";
      ctx2.shadowBlur = on ? blur * opts.px * s : 0;
    };
    switch (skin.eyes) {
      case "pair":
        for (const side of [-1, 1]) {
          const e = at(sp, 14 * s, side * 3.2 * s);
          dot(e, 2.4 * s, skin.colors.eye);
          glint(e, 2.4 * s);
        }
        break;
      case "quad":
        for (const k of [-1.5, -0.5, 0.5, 1.5]) {
          const e = at(sp, (14.2 - Math.abs(k) * 1.3) * s, k * 2.7 * s);
          dot(e, 1.55 * s, skin.colors.eye);
          glint(e, 1.55 * s);
        }
        break;
      case "visor": {
        const rx = geo.cephR.x;
        const ry = geo.cephR.y;
        ctx2.save();
        ctx2.translate(geo.ceph.x, geo.ceph.y);
        ctx2.rotate(sp.heading);
        ctx2.fillStyle = skin.colors.eye;
        ctx2.beginPath();
        ctx2.roundRect(rx * 0.32, -ry * 0.74, rx * 0.5, ry * 1.48, 2.2 * s);
        ctx2.fill();
        ctx2.restore();
        glow(true, 6);
        for (const side of [-1, 1]) dot(at(sp, 6 * s + rx * 0.57, side * ry * 0.38), 1.5 * s, skin.colors.accent);
        glow(false, 0);
        break;
      }
      case "cyclops": {
        const e = at(sp, 6 * s + geo.cephR.x * 0.42, 0);
        const r = geo.cephR.y * 0.5;
        dot(e, r, skin.colors.eye);
        ctx2.beginPath();
        ctx2.arc(e.x, e.y, r * 0.62, 0, Math.PI * 2);
        ctx2.strokeStyle = skin.colors.accent;
        ctx2.lineWidth = 1.1 * s;
        glow(skin.glow > 0, 8);
        ctx2.stroke();
        glow(false, 0);
        glint(e, r);
        break;
      }
    }
  }

  // src/spider/skins.ts
  var SKINS = [
    {
      id: "blueprint",
      name: "Blueprint",
      blurb: "The original rig: hips, knees, foot targets and steps in flight.",
      scale: 1.15,
      legLength: 1,
      legWidth: 1,
      abdomen: { length: 1, width: 1 },
      body: "ellipse",
      joints: "circle",
      eyes: "pair",
      colors: { legs: ["#f4b400", "#ff4f86"], ink: "#3f9cf0", fill: "rgba(63, 156, 240, 0.22)", eye: "#0d1b2a", accent: "#f4b400" },
      glow: 0,
      halo: false
    },
    {
      id: "boxy",
      name: "Boxy",
      blurb: "Built from rectangles, with square rivets and a robot visor.",
      scale: 1.15,
      legLength: 0.95,
      legWidth: 1.3,
      abdomen: { length: 1, width: 1.05 },
      body: "rect",
      joints: "square",
      eyes: "visor",
      colors: { legs: ["#ff6b4a", "#00b3a4"], ink: "#8b5cf6", fill: "rgba(139, 92, 246, 0.2)", eye: "#1b1530", accent: "#5ef2ff" },
      glow: 0,
      halo: false
    },
    {
      id: "hexa",
      name: "Hexa",
      blurb: "Hexagonal plates and diamond joints, like a low-poly bug.",
      scale: 1.15,
      legLength: 1.05,
      legWidth: 1.05,
      abdomen: { length: 1.12, width: 0.95 },
      body: "hex",
      joints: "diamond",
      eyes: "quad",
      colors: { legs: ["#6fb800", "#3f9cf0"], ink: "#14b8a6", fill: "rgba(20, 184, 166, 0.2)", eye: "#0b2420", accent: "#7cc800" },
      glow: 0,
      halo: false
    },
    {
      id: "neon",
      name: "Neon",
      blurb: "Glowing diamonds and one big eye. Best at night.",
      scale: 1.1,
      legLength: 1.1,
      legWidth: 0.95,
      abdomen: { length: 1.05, width: 0.92 },
      body: "diamond",
      joints: "circle",
      eyes: "cyclops",
      colors: { legs: ["#ff2bd6", "#00c8ff"], ink: "#a979ff", fill: "rgba(169, 121, 255, 0.16)", eye: "#14002a", accent: "#00e5ff" },
      glow: 1,
      halo: false
    },
    {
      id: "stilts",
      name: "Stilts",
      blurb: "A tiny round body on eight enormous legs.",
      scale: 0.95,
      legLength: 2,
      legWidth: 0.8,
      abdomen: { length: 0.8, width: 0.85 },
      body: "circle",
      joints: "circle",
      eyes: "pair",
      colors: { legs: ["#8b5cf6", "#ff8a00"], ink: "#f97316", fill: "rgba(249, 115, 22, 0.18)", eye: "#2a1405", accent: "#8b5cf6" },
      glow: 0,
      halo: false
    },
    {
      id: "sketch",
      name: "Sketch",
      blurb: "Drawn in ink, like a margin doodle that walked off.",
      scale: 1.15,
      legLength: 1,
      legWidth: 1.05,
      abdomen: { length: 1, width: 1 },
      body: "ellipse",
      joints: "circle",
      eyes: "pair",
      colors: { legs: ["#23253a", "#6b6e88"], ink: "#23253a", fill: "rgba(255, 255, 255, 0.8)", eye: "#23253a", accent: "#ff4f86" },
      glow: 0,
      halo: true
    }
  ];
  var skinById = (id) => SKINS.find((s) => s.id === id) ?? SKINS[0];

  // src/main.ts
  var canvas = document.getElementById("stage");
  var ctx = canvas.getContext("2d");
  var dpr = Math.min(2, window.devicePixelRatio || 1);
  var W = 0;
  var H = 0;
  var zoom = 1.65;
  var resize = () => {
    W = canvas.clientWidth;
    H = canvas.clientHeight;
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
  };
  resize();
  addEventListener("resize", resize);
  var spider = new Spider({ x: 0, y: 40 }, skinById("blueprint"));
  var mouse = null;
  var toWorld = (clientX, clientY) => {
    const rect = canvas.getBoundingClientRect();
    const x = clientX - rect.left;
    const y = clientY - rect.top;
    return { x: (x - W / 2) / zoom, y: (y - H / 2) / zoom };
  };
  canvas.addEventListener("pointermove", (e) => {
    mouse = toWorld(e.clientX, e.clientY);
  });
  canvas.addEventListener("pointerdown", (e) => {
    spider.jumpTo(toWorld(e.clientX, e.clientY));
  });
  canvas.addEventListener("pointerleave", () => {
    mouse = null;
  });
  var PROFILE_LINES = [
    { text: "#!/usr/bin/python", color: "#8b949e" },
    { text: "# -*- coding: utf-8 -*-", color: "#8b949e" },
    { text: "" },
    { text: "class SoftwareEngineer:" },
    { text: "    def __init__(self):" },
    { text: '        self.name = "Alver-Lin"' },
    { text: '        self.role = "Software Engineer"' },
    { text: '        self.language_spoken = ["zh_CN", "en_US"]' },
    { text: "" },
    { text: '        self.current_project = "seeUtoken"' },
    { text: '        self.currently_learning = "agenttest"' },
    { text: "        self.interests = [" },
    { text: '            "Test Development",' },
    { text: '            "Artificial Intelligence",' },
    { text: '            "Open Source Tools",' },
    { text: '            "AI Agent Evaluation",' },
    { text: "        ]" },
    { text: '        self.open_to_discuss = "AI Agent Evaluation"' },
    { text: "" },
    { text: "    def say_hi(self):" },
    { text: '        print("Thanks for dropping by!")' },
    { text: '        print("Hope you find some of my work interesting.")' },
    { text: "" },
    { text: "    def now_doing(self):" },
    { text: '        print(f"Working on: {self.current_project}")' },
    { text: '        print(f"Learning: {self.currently_learning}")' }
  ];
  function drawCard() {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, W, H);
    const cardW = Math.min(920, W - 48);
    const cardH = Math.min(980, H - 48);
    const cardX = (W - cardW) / 2;
    const cardY = (H - cardH) / 2;
    const r = 16;
    ctx.beginPath();
    ctx.moveTo(cardX + r, cardY);
    ctx.arcTo(cardX + cardW, cardY, cardX + cardW, cardY + cardH, r);
    ctx.arcTo(cardX + cardW, cardY + cardH, cardX, cardY + cardH, r);
    ctx.arcTo(cardX, cardY + cardH, cardX, cardY, r);
    ctx.arcTo(cardX, cardY, cardX + cardW, cardY, r);
    ctx.closePath();
    ctx.fillStyle = "#0d1117";
    ctx.fill();
    ctx.strokeStyle = "#30363d";
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cardX, cardY + 45);
    ctx.lineTo(cardX + cardW, cardY + 45);
    ctx.strokeStyle = "#30363d";
    ctx.stroke();
    const dots = ["#ff6058", "#febc2e", "#28c840"];
    dots.forEach((c, i) => {
      ctx.beginPath();
      ctx.arc(cardX + 27 + i * 18, cardY + 23, 5, 0, Math.PI * 2);
      ctx.fillStyle = c;
      ctx.fill();
    });
    ctx.fillStyle = "#8b949e";
    ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.fillText("alver-lin \u2014 profile.py", cardX + cardW / 2, cardY + 28);
    ctx.textAlign = "left";
    ctx.fillStyle = "#8b949e";
    ctx.font = "14px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.fillText("$ cat profile.py", cardX + 48, cardY + 84);
    ctx.fillStyle = "#58a6ff";
    ctx.font = "700 36px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.fillText("> Hello, I am Alver-Lin", cardX + 47, cardY + 140);
    ctx.beginPath();
    ctx.moveTo(cardX + 65, cardY + 170);
    ctx.lineTo(cardX + 65, cardY + cardH - 24);
    ctx.strokeStyle = "#21262d";
    ctx.stroke();
    const lineH = 22;
    const startY = cardY + 190;
    PROFILE_LINES.forEach((line, i) => {
      const y = startY + i * lineH;
      if (y > cardY + cardH - 30) return;
      ctx.fillStyle = "#768390";
      ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
      ctx.textAlign = "right";
      ctx.fillText(String(i + 1), cardX + 51, y);
      ctx.textAlign = "left";
      ctx.fillStyle = line.color || "#c9d1d9";
      ctx.font = "15px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
      ctx.fillText(line.text, cardX + 78, y);
    });
    return { cardX, cardY, cardW, cardH };
  }
  function loop(now) {
    const last = loop._last ?? now;
    loop._last = now;
    const dt = Math.min(0.05, (now - last) / 1e3);
    if (mouse) spider.goal = mouse;
    spider.update(dt);
    drawCard();
    ctx.setTransform(
      dpr * zoom,
      0,
      0,
      dpr * zoom,
      dpr * W / 2,
      dpr * H / 2
    );
    drawSpider(ctx, spider, { ...DEFAULT_DRAW, px: zoom * dpr, shadow: 0.85 });
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);
})();
