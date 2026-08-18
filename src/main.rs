//! Standalone, safety-gated bridge for external XYZ velocity commands.
//!
//! This is a separate Cargo package. The existing rustbot_cntrl package is
//! untouched. The bridge accepts JSON-lines commands from the Python adapter,
//! and only the explicit `--mode live --arm` path opens ABB EGM.

use anyhow::{bail, Context, Result};
use prost::Message;
use serde::Deserialize;
use serde_json::{json, Value};
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream, UdpSocket};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[path = "../../src/control/egm_control/abb_egm.rs"]
mod abb_egm;

use abb_egm::egm_header::MessageType;
use abb_egm::{
    EgmCartesian, EgmCartesianSpeed, EgmClock, EgmHeader, EgmPlanned, EgmPose, EgmQuaternion,
    EgmRobot, EgmSensor, EgmSpeedRef,
};

const DEFAULT_TCP_BIND: &str = "127.0.0.1:45890";
const DEFAULT_ABB_IP: &str = "192.168.125.1";
const DEFAULT_ABB_PORT: u16 = 8888;
const DEFAULT_EGM_BIND: &str = "192.168.125.206:6510";
const DEFAULT_WATCHDOG_MS: u64 = 250;
const DEFAULT_MAX_SPEED_MM_S: f64 = 25.0;

#[derive(Clone, Copy, Debug)]
struct CommandState {
    velocity_mm_s: [f64; 3],
    received: Instant,
    sequence: u64,
    valid: bool,
}

impl Default for CommandState {
    fn default() -> Self {
        Self {
            velocity_mm_s: [0.0; 3],
            received: Instant::now(),
            sequence: 0,
            valid: false,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Telemetry {
    eef_pos_mm: [f64; 3],
    eef_quaternion_wxyz: [f64; 4],
    egm_feedback: bool,
    egm_timestamp: u32,
    last_feedback: Instant,
}

struct SharedState {
    command: Mutex<CommandState>,
    telemetry: Mutex<Telemetry>,
    fake_last_update: Mutex<Instant>,
}

impl SharedState {
    fn new(fake_eef_mm: [f64; 3]) -> Self {
        Self {
            command: Mutex::new(CommandState::default()),
            telemetry: Mutex::new(Telemetry {
                eef_pos_mm: fake_eef_mm,
                eef_quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
                egm_feedback: false,
                egm_timestamp: 0,
                last_feedback: Instant::now(),
            }),
            fake_last_update: Mutex::new(Instant::now()),
        }
    }
}

#[derive(Debug)]
struct BridgeConfig {
    mode: String,
    arm: bool,
    tcp_bind: String,
    abb_ip: String,
    abb_port: u16,
    egm_bind: String,
    watchdog: Duration,
    max_speed_mm_s: f64,
}

#[derive(Debug, Deserialize)]
struct WireCommand {
    #[serde(rename = "type")]
    kind: String,
    sequence: Option<u64>,
    timestamp_ns: Option<u64>,
    valid_until_ns: Option<u64>,
    frame: Option<String>,
    velocity_mm_s: Option<[f64; 3]>,
    rotation_velocity_rad_s: Option<[f64; 3]>,
    valid: Option<bool>,
}

fn usage() {
    println!(
        "Usage: real_robot_xyz_velocity_bridge [--mode dry-run|live] [--arm]\n\
         [--tcp-bind ADDR] [--abb-ip IP] [--abb-port PORT]\n\
         [--egm-bind IP:PORT] [--watchdog-ms N] [--max-speed-mm-s N]\n\
         [--fake-eef-mm X,Y,Z]"
    );
}

fn parse_triplet(value: &str) -> Result<[f64; 3]> {
    let values: Vec<f64> = value
        .split(',')
        .map(|part| part.trim().parse::<f64>())
        .collect::<std::result::Result<Vec<_>, _>>()
        .context("expected comma-separated numeric triplet")?;
    if values.len() != 3 || values.iter().any(|value| !value.is_finite()) {
        bail!("expected three finite values")
    }
    Ok([values[0], values[1], values[2]])
}

fn parse_args() -> Result<(BridgeConfig, [f64; 3])> {
    let args: Vec<String> = std::env::args().collect();
    let mut mode = "dry-run".to_string();
    let mut arm = false;
    let mut tcp_bind = DEFAULT_TCP_BIND.to_string();
    let mut abb_ip = DEFAULT_ABB_IP.to_string();
    let mut abb_port = DEFAULT_ABB_PORT;
    let mut egm_bind = DEFAULT_EGM_BIND.to_string();
    let mut watchdog_ms = DEFAULT_WATCHDOG_MS;
    let mut max_speed = DEFAULT_MAX_SPEED_MM_S;
    let mut fake_eef = [300.0, 150.0, 600.0];
    let mut index = 1;
    while index < args.len() {
        match args[index].as_str() {
            "--help" | "-h" => {
                usage();
                std::process::exit(0);
            }
            "--arm" => arm = true,
            "--mode" => {
                index += 1;
                mode = args.get(index).context("--mode needs a value")?.clone();
            }
            "--tcp-bind" => {
                index += 1;
                tcp_bind = args.get(index).context("--tcp-bind needs a value")?.clone();
            }
            "--abb-ip" => {
                index += 1;
                abb_ip = args.get(index).context("--abb-ip needs a value")?.clone();
            }
            "--abb-port" => {
                index += 1;
                abb_port = args
                    .get(index)
                    .context("--abb-port needs a value")?
                    .parse()?;
            }
            "--egm-bind" => {
                index += 1;
                egm_bind = args.get(index).context("--egm-bind needs a value")?.clone();
            }
            "--watchdog-ms" => {
                index += 1;
                watchdog_ms = args
                    .get(index)
                    .context("--watchdog-ms needs a value")?
                    .parse()?;
            }
            "--max-speed-mm-s" => {
                index += 1;
                max_speed = args
                    .get(index)
                    .context("--max-speed-mm-s needs a value")?
                    .parse()?;
            }
            "--fake-eef-mm" => {
                index += 1;
                fake_eef = parse_triplet(args.get(index).context("--fake-eef-mm needs a value")?)?;
            }
            other => bail!("unknown argument {other}"),
        }
        index += 1;
    }
    if mode != "dry-run" && mode != "live" {
        bail!("--mode must be dry-run or live")
    }
    if mode == "live" && !arm {
        bail!("live mode requires explicit --arm")
    }
    if !max_speed.is_finite() || max_speed <= 0.0 {
        bail!("max speed must be positive")
    }
    Ok((
        BridgeConfig {
            mode,
            arm,
            tcp_bind,
            abb_ip,
            abb_port,
            egm_bind,
            watchdog: Duration::from_millis(watchdog_ms),
            max_speed_mm_s: max_speed,
        },
        fake_eef,
    ))
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn finite_triplet(value: [f64; 3]) -> bool {
    value.iter().all(|component| component.is_finite())
}

fn egm_feedback_fresh(shared: &SharedState, watchdog: Duration) -> bool {
    let telemetry = shared.telemetry.lock().expect("telemetry poisoned");
    telemetry.egm_feedback && telemetry.last_feedback.elapsed() <= watchdog
}

fn integrate_fake(shared: &SharedState, watchdog: Duration) {
    let now = Instant::now();
    let mut last = shared.fake_last_update.lock().expect("fake clock poisoned");
    let dt = now.duration_since(*last).as_secs_f64();
    *last = now;
    let command = *shared.command.lock().expect("command poisoned");
    let mut telemetry = shared.telemetry.lock().expect("telemetry poisoned");
    let velocity = if command.valid && command.received.elapsed() <= watchdog {
        command.velocity_mm_s
    } else {
        [0.0; 3]
    };
    for axis in 0..3 {
        telemetry.eef_pos_mm[axis] += velocity[axis] * dt;
    }
}

fn telemetry_json(shared: &SharedState, config: &BridgeConfig) -> Value {
    let telemetry = *shared.telemetry.lock().expect("telemetry poisoned");
    let command = *shared.command.lock().expect("command poisoned");
    let command_stale = !command.valid || command.received.elapsed() > config.watchdog;
    let feedback_stale = config.mode == "live" && !egm_feedback_fresh(shared, config.watchdog);
    json!({
        "type": "status",
        "eef_pos_mm": telemetry.eef_pos_mm,
        "eef_quaternion_wxyz": telemetry.eef_quaternion_wxyz,
        "egm_feedback": telemetry.egm_feedback,
        "egm_feedback_fresh": telemetry.egm_feedback && !feedback_stale,
        "egm_timestamp": telemetry.egm_timestamp,
        "watchdog_active": command_stale || feedback_stale,
    })
}

fn validate_and_store(
    command: WireCommand,
    shared: &SharedState,
    config: &BridgeConfig,
) -> Result<Value> {
    if command.kind != "velocity_command" {
        bail!("expected velocity_command")
    }
    if command.frame.as_deref() != Some("abb_base") {
        bail!("velocity command frame must be abb_base")
    }
    if let Some(rotation) = command.rotation_velocity_rad_s {
        if !finite_triplet(rotation) || rotation.iter().any(|value| value.abs() > 1e-9) {
            bail!("orientation control is disabled; rotation velocity must be zero")
        }
    }
    let sequence = command.sequence.unwrap_or(0);
    let valid = command.valid.unwrap_or(true);
    let mut velocity = command.velocity_mm_s.unwrap_or([0.0; 3]);
    if !finite_triplet(velocity) {
        bail!("velocity_mm_s contains non-finite values")
    }
    if valid && command.valid_until_ns.is_none() {
        bail!("valid velocity command requires valid_until_ns")
    }
    if valid {
        let timestamp_ns = command
            .timestamp_ns
            .context("valid velocity command requires timestamp_ns")?;
        let valid_until_ns = command.valid_until_ns.expect("checked above");
        if valid_until_ns <= timestamp_ns {
            bail!("valid_until_ns must be later than timestamp_ns")
        }
    }
    if config.mode == "live" && valid && !egm_feedback_fresh(shared, config.watchdog) {
        bail!("no fresh ABB EGM feedback; live command rejected")
    }
    let mut state = shared.command.lock().expect("command poisoned");
    if valid && sequence != 0 && sequence <= state.sequence {
        bail!("non-increasing command sequence")
    }
    let now = now_ns();
    let deadline_expired = command
        .valid_until_ns
        .map(|deadline| deadline <= now)
        .unwrap_or(false);
    if deadline_expired {
        velocity = [0.0; 3];
    }
    for component in &mut velocity {
        *component = component.clamp(-config.max_speed_mm_s, config.max_speed_mm_s);
    }
    state.velocity_mm_s = if valid { velocity } else { [0.0; 3] };
    state.received = Instant::now();
    if valid && sequence != 0 {
        state.sequence = sequence;
    }
    state.valid = valid && !deadline_expired;
    Ok(json!({
        "type": "ack",
        "accepted": true,
        "sequence": sequence,
        "velocity_mm_s": state.velocity_mm_s,
        "watchdog_active": !state.valid,
    }))
}

fn handle_client(
    mut stream: TcpStream,
    shared: Arc<SharedState>,
    config: Arc<BridgeConfig>,
) -> Result<()> {
    stream.set_nodelay(true)?;
    let reader_stream = stream.try_clone()?;
    let reader = BufReader::new(reader_stream);
    for line in reader.lines() {
        let line = line.context("read bridge request")?;
        if line.trim().is_empty() {
            continue;
        }
        let value: Value = serde_json::from_str(&line).context("invalid JSON request")?;
        let kind = value
            .get("type")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string();
        let response_result: Result<Value> = (|| {
            Ok(match kind.as_str() {
                "hello" => json!({
                    "type": "hello_ack",
                    "protocol": "legacy8d_xyz_velocity_v1",
                    "mode": config.mode.as_str(),
                    "armed": config.arm,
                    "orientation_control": false,
                    "status": telemetry_json(&shared, &config),
                }),
                "status" => {
                    if config.mode == "dry-run" {
                        integrate_fake(&shared, config.watchdog);
                    }
                    telemetry_json(&shared, &config)
                }
                "velocity_command" => {
                    let command: WireCommand = serde_json::from_value(value)?;
                    if config.mode == "dry-run" {
                        let result = validate_and_store(command, &shared, &config)?;
                        integrate_fake(&shared, config.watchdog);
                        result
                    } else if config.arm {
                        validate_and_store(command, &shared, &config)?
                    } else {
                        bail!("bridge is not armed")
                    }
                }
                _ => bail!("unknown request type {kind}"),
            })
        })();
        let response = match response_result {
            Ok(value) => value,
            Err(error) => json!({"type": "error", "message": format!("{error:#}")}),
        };
        stream.write_all(serde_json::to_string(&response)?.as_bytes())?;
        stream.write_all(b"\n")?;
        stream.flush()?;
    }
    let mut command = shared.command.lock().expect("command poisoned");
    command.velocity_mm_s = [0.0; 3];
    command.valid = false;
    Ok(())
}

struct AbbTcp {
    stream: TcpStream,
}

impl AbbTcp {
    fn connect(ip: &str, port: u16) -> Result<Self> {
        let stream = TcpStream::connect((ip, port)).context("connect ABB command socket")?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        stream.set_write_timeout(Some(Duration::from_secs(5)))?;
        Ok(Self { stream })
    }

    fn request(&mut self, command: &str) -> Result<String> {
        self.stream.write_all(command.as_bytes())?;
        let mut response = Vec::new();
        loop {
            let mut byte = [0u8; 1];
            self.stream.read_exact(&mut byte)?;
            response.push(byte[0]);
            if byte[0] == b'!' {
                break;
            }
        }
        response.pop();
        Ok(String::from_utf8(response)?)
    }
}

use std::io::Read;

fn egm_time() -> (u64, u64) {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    (duration.as_secs(), u64::from(duration.subsec_micros()))
}

fn make_sensor(
    seq: u32,
    time: (u64, u64),
    position_mm: [f64; 3],
    quaternion: [f64; 4],
    velocity: [f64; 3],
) -> EgmSensor {
    let timestamp = time.0.wrapping_mul(1_000).wrapping_add(time.1 / 1_000) as u32;
    EgmSensor {
        header: Some(EgmHeader {
            seqno: Some(seq),
            tm: Some(timestamp),
            mtype: Some(MessageType::MsgtypeCorrection.into()),
        }),
        planned: Some(EgmPlanned {
            joints: None,
            cartesian: Some(EgmPose {
                pos: Some(EgmCartesian {
                    x: position_mm[0],
                    y: position_mm[1],
                    z: position_mm[2],
                }),
                orient: Some(EgmQuaternion {
                    u0: quaternion[0],
                    u1: quaternion[1],
                    u2: quaternion[2],
                    u3: quaternion[3],
                }),
                euler: None,
            }),
            external_joints: None,
            time: Some(EgmClock {
                sec: time.0,
                usec: time.1,
            }),
        }),
        speed_ref: Some(EgmSpeedRef {
            joints: None,
            cartesians: Some(EgmCartesianSpeed {
                value: vec![velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0],
            }),
            external_joints: None,
        }),
    }
}

fn update_egm_telemetry(shared: &SharedState, robot: &EgmRobot) {
    let Some(feedback) = robot.feed_back.as_ref() else {
        return;
    };
    let Some(cartesian) = feedback.cartesian.as_ref() else {
        return;
    };
    let Some(position) = cartesian.pos.as_ref() else {
        return;
    };
    let Some(orientation) = cartesian.orient.as_ref() else {
        return;
    };
    let mut telemetry = shared.telemetry.lock().expect("telemetry poisoned");
    telemetry.eef_pos_mm = [position.x, position.y, position.z];
    telemetry.eef_quaternion_wxyz = [
        orientation.u0,
        orientation.u1,
        orientation.u2,
        orientation.u3,
    ];
    telemetry.egm_feedback = true;
    telemetry.egm_timestamp = robot
        .header
        .as_ref()
        .and_then(|header| header.tm)
        .unwrap_or(0);
    telemetry.last_feedback = Instant::now();
}

fn run_egm(shared: Arc<SharedState>, config: Arc<BridgeConfig>) -> Result<()> {
    // Match the established rustbot order: bind the host UDP endpoint before
    // asking RAPID to create and start its EGM connection.
    let socket = UdpSocket::bind(&config.egm_bind).context("bind EGM UDP socket")?;
    socket.set_read_timeout(Some(Duration::from_millis(4)))?;
    let mut abb = AbbTcp::connect(&config.abb_ip, config.abb_port)?;
    abb.request("EGPS:0")?;
    abb.request("EGSS:0")?;
    println!("[rust-bridge] EGM UDP waiting on {}", config.egm_bind);
    let mut buffer = vec![0u8; 4096];
    let (size, peer) = socket
        .recv_from(&mut buffer)
        .context("wait for ABB EGM feedback")?;
    socket.connect(peer)?;
    let first = EgmRobot::decode(&buffer[..size])?;
    update_egm_telemetry(&shared, &first);
    let mut sequence = 0u32;
    loop {
        match socket.recv(&mut buffer) {
            Ok(size) => {
                let robot = EgmRobot::decode(&buffer[..size])?;
                update_egm_telemetry(&shared, &robot);
            }
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock
                    || error.kind() == std::io::ErrorKind::TimedOut => {}
            Err(error) => return Err(error.into()),
        }
        let command = *shared.command.lock().expect("command poisoned");
        let mut velocity = if command.valid
            && command.received.elapsed() <= config.watchdog
            && egm_feedback_fresh(&shared, config.watchdog)
        {
            command.velocity_mm_s
        } else {
            [0.0; 3]
        };


        //JOE ADDED--------------------
        //Maximum speed the robot is allowed to move
        const MAX_SPEED : f64 = 1.0;
        //Clamp the velocity speeds
        velocity[0] = velocity[0].clamp(-MAX_SPEED, MAX_SPEED);
        velocity[1] = velocity[1].clamp(-MAX_SPEED, MAX_SPEED);
        velocity[2] = velocity[2].clamp(-MAX_SPEED, MAX_SPEED);    
        //JOE ADDED--------------------
        
        let telemetry = *shared.telemetry.lock().expect("telemetry poisoned");
        socket.send(
            &make_sensor(
                sequence,
                egm_time(),
                telemetry.eef_pos_mm,
                telemetry.eef_quaternion_wxyz,
                velocity,
            )
            .encode_to_vec(),
        )?;
        sequence = sequence.wrapping_add(1);
    }
}

fn main() -> Result<()> {
    let (config, fake_eef_mm) = parse_args()?;
    let config = Arc::new(config);
    let shared = Arc::new(SharedState::new(fake_eef_mm));
    if config.mode == "live" && config.arm {
        let egm_shared = Arc::clone(&shared);
        let egm_config = Arc::clone(&config);
        thread::spawn(move || {
            if let Err(error) = run_egm(egm_shared.clone(), egm_config) {
                eprintln!("[rust-bridge] EGM stopped: {error:#}");
                let mut command = egm_shared.command.lock().expect("command poisoned");
                command.velocity_mm_s = [0.0; 3];
                command.valid = false;
            }
        });
    } else {
        println!("[rust-bridge] dry-run: no ABB socket or EGM is opened");
    }
    let listener = TcpListener::bind(&config.tcp_bind)
        .with_context(|| format!("bind bridge TCP {}", config.tcp_bind))?;
    println!("[rust-bridge] LISTENING on {}", config.tcp_bind);
    for incoming in listener.incoming() {
        match incoming {
            Ok(stream) => {
                let client_shared = Arc::clone(&shared);
                let client_config = Arc::clone(&config);
                if let Err(error) = handle_client(stream, client_shared, client_config) {
                    eprintln!("[rust-bridge] client stopped: {error:#}");
                }
            }
            Err(error) => eprintln!("[rust-bridge] accept error: {error}"),
        }
    }
    Ok(())
}
