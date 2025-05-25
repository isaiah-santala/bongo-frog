extends CharacterBody3D
class_name FirstPersonController

# Movement settings
@export var walk_speed: float = 8.0
@export var sprint_speed: float = 12.0
@export var swim_speed: float = 6.0
@export var air_speed_multiplier: float = 0.8
@export var instant_acceleration: bool = true
@export var ground_friction: float = 20.0
@export var air_friction: float = 2.0

# Jump settings
@export var jump_velocity: float = 6.0
@export var coyote_time: float = 0.08
@export var air_control: float = 0.3
@export var fall_multiplier: float = 2.5
@export var low_jump_multiplier: float = 2.0

# Swimming settings
@export var swim_surface_level: float = 0.0
@export var buoyancy_force: float = 2.0
@export var water_drag: float = 5.0
@export var camera_relative_swimming: bool = true # New option to toggle camera-relative swimming

# Mouse sensitivity
@export var mouse_sensitivity: float = 0.002
@export var max_look_angle: float = 90.0

# Get the gravity from the project settings
var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")

# Internal variables
var is_sprinting: bool = false
var is_swimming: bool = false
var is_head_underwater: bool = false
var is_on_surface: bool = false
var coyote_timer: float = 0.0

# Camera references
@export var camera_pivot: Node3D
@export var camera: Camera3D

# Water detection
var feet_water_bodies: Array[Area3D] = []
@export var feet_collider: Area3D
var head_water_bodies: Array[Area3D] = []
@export var head_collider: Area3D
@export var underwater_effect: Control

func _ready():
    assert(camera_pivot, "Camera pivot node is not assigned.")
    assert(camera, "Camera node is not assigned.")
    assert(feet_collider, "Feet collider node is not assigned.")
    assert(head_collider, "Head collider node is not assigned.")
    assert(underwater_effect, "Underwater effect node is not assigned.")
    # Capture mouse
    Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
    
    feet_collider.area_entered.connect(_on_feet_entered_area)
    feet_collider.area_exited.connect(_on_feet_exited_area)
    head_collider.area_entered.connect(_on_head_entered_area)
    head_collider.area_exited.connect(_on_head_exited_area)

func _input(event):
    # Handle mouse look
    if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
        handle_mouse_look(event.relative)
    
    # Toggle mouse capture
    if event.is_action_pressed("ui_cancel"):
        if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
            Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
        else:
            Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func _physics_process(delta):
    update_movement_state()
    handle_movement(delta)
    handle_jumping()
    handle_swimming(delta)
    apply_movement(delta)
    update_camera_effects()

func handle_mouse_look(relative_motion: Vector2):
    # Rotate the body horizontally
    rotate_y(-relative_motion.x * mouse_sensitivity)
    
    # Rotate the camera vertically
    if camera_pivot:
        camera_pivot.rotate_x(-relative_motion.y * mouse_sensitivity)
        camera_pivot.rotation.x = clamp(camera_pivot.rotation.x,
            deg_to_rad(-max_look_angle), deg_to_rad(max_look_angle))

func update_movement_state():
    # Check if sprinting
    is_sprinting = Input.is_action_pressed("sprint") and not is_swimming
    
    # Update swimming state
    is_swimming = feet_water_bodies.size() > 0
    is_head_underwater = head_water_bodies.size() > 0
    is_on_surface = is_swimming and global_position.y >= swim_surface_level - 0.5

func handle_movement(delta):
    # Get input direction
    var input_dir = Input.get_vector("move-left", "move-right", "move-forward", "move-backward")
    
    # Calculate direction based on swimming mode
    var direction: Vector3
    if is_swimming and camera_relative_swimming:
        # Camera-relative movement for swimming
        direction = get_camera_relative_direction(input_dir)
    else:
        # Standard horizontal movement (ignores camera pitch)
        direction = (transform.basis * Vector3(input_dir.x, 0, input_dir.y)).normalized()
    
    # Determine target speed
    var target_speed: float
    if is_swimming:
        target_speed = swim_speed
    elif is_sprinting:
        target_speed = sprint_speed
    else:
        target_speed = walk_speed
    
    # Apply air movement penalty
    if not is_on_floor() and not is_swimming:
        target_speed *= air_speed_multiplier
    
    # Apply movement - arcade style
    if direction != Vector3.ZERO:
        if instant_acceleration and is_on_floor():
            # Instant acceleration for snappy ground movement
            velocity.x = direction.x * target_speed
            velocity.z = direction.z * target_speed
            # Only apply Y velocity when swimming with camera-relative movement
            if is_swimming and camera_relative_swimming:
                velocity.y = direction.y * target_speed
        else:
            # Gradual acceleration for air movement or when instant_acceleration is off
            var accel_rate = 15.0 if is_on_floor() else 8.0
            var target_velocity = direction * target_speed
            velocity.x = move_toward(velocity.x, target_velocity.x, accel_rate * delta)
            velocity.z = move_toward(velocity.z, target_velocity.z, accel_rate * delta)
            # Apply Y velocity for swimming
            if is_swimming and camera_relative_swimming:
                velocity.y = move_toward(velocity.y, target_velocity.y, accel_rate * delta)
    else:
        # Apply friction based on surface
        var friction_force: float
        if is_swimming:
            friction_force = water_drag
        elif is_on_floor():
            friction_force = ground_friction
        else:
            friction_force = air_friction
        
        velocity.x = move_toward(velocity.x, 0, friction_force * delta)
        velocity.z = move_toward(velocity.z, 0, friction_force * delta)
        
        # Apply friction to Y velocity when swimming with camera-relative movement
        if is_swimming and camera_relative_swimming:
            velocity.y = move_toward(velocity.y, 0, friction_force * delta * 0.5) # Reduced Y friction for better swimming feel

func get_camera_relative_direction(input_dir: Vector2) -> Vector3:
    # Get the camera's forward direction (including pitch)
    var camera_forward = - camera_pivot.global_transform.basis.z
    var camera_right = camera_pivot.global_transform.basis.x
    
    # Create movement direction relative to camera orientation
    var forward_movement = camera_forward * -input_dir.y # Forward/backward
    var right_movement = camera_right * input_dir.x # Left/right
    
    return (forward_movement + right_movement).normalized()

func handle_jumping():
    # Update coyote timer
    if is_on_floor():
        coyote_timer = coyote_time
    elif coyote_timer > 0:
        coyote_timer -= get_physics_process_delta_time()
    
    # Handle jump input
    if Input.is_action_just_pressed("jump"):
        if is_swimming:
            if camera_relative_swimming:
                # When using camera-relative swimming, jump adds upward boost
                velocity.y += jump_velocity * 0.6
            else:
                # Original swimming up behavior
                velocity.y = jump_velocity * 0.8
        elif coyote_timer > 0:
            # Regular jump - snappy and controlled height
            velocity.y = jump_velocity
            coyote_timer = 0
    
    # Variable jump height - release jump early for shorter jumps
    if Input.is_action_just_released("jump") and velocity.y > 0 and not is_swimming:
        velocity.y *= 0.5

func handle_swimming(delta):
    if not is_swimming:
        return
    
    # Apply buoyancy when underwater - more responsive
    if not is_on_surface:
        velocity.y += buoyancy_force * delta * 1.5
    
    # Check for swimming up/down inputs - treat jump and swim-up identically, crouch and swim-down identically
    var swim_up_pressed = Input.is_action_pressed("jump") or Input.is_action_pressed("swim-up")
    var swim_down_pressed = Input.is_action_pressed("crouch") or Input.is_action_pressed("swim-down")
    
    # Handle swimming controls when not using camera-relative movement
    if not camera_relative_swimming:
        # Original swimming up/down controls
        if swim_up_pressed:
            velocity.y = swim_speed * 0.8
        elif swim_down_pressed:
            velocity.y = - swim_speed * 0.8
        else:
            # Quick settle to neutral buoyancy
            if not is_on_surface:
                velocity.y = move_toward(velocity.y, buoyancy_force * 0.2, water_drag * delta * 2.0)
            else:
                velocity.y = move_toward(velocity.y, 0, water_drag * delta * 3.0)
    else:
        # With camera-relative swimming, vertical controls still work
        if swim_down_pressed:
            velocity.y -= swim_speed * delta * 2.0 # Force downward
        elif swim_up_pressed:
            # Treat jump and swim-up identically - no special case for jump anymore
            velocity.y += swim_speed * delta * 1.5 # Force upward
        else:
            # Gentle buoyancy when not actively moving up
            if not is_on_surface:
                velocity.y = move_toward(velocity.y, buoyancy_force * 0.1, water_drag * delta)
            else:
                velocity.y = move_toward(velocity.y, 0, water_drag * delta * 2.0)

func apply_movement(delta):
    # Apply enhanced gravity when not swimming
    if not is_swimming and not is_on_floor():
        if velocity.y < 0:
            # Falling - apply stronger gravity for faster descent
            velocity.y -= gravity * fall_multiplier * delta
        elif velocity.y > 0 and not Input.is_action_pressed("jump"):
            # Rising but not holding jump - apply extra gravity for low jump
            velocity.y -= gravity * low_jump_multiplier * delta
        else:
            # Normal gravity when rising and holding jump
            velocity.y -= gravity * delta
    
    # Apply water drag when swimming
    if is_swimming:
        velocity *= (1.0 - water_drag * delta * 0.1)
    
    # Move the character
    move_and_slide()

func update_camera_effects():
    underwater_effect.visible = is_head_underwater

# Water detection functions
func _on_feet_entered_area(area: Area3D):
    if area.is_in_group("water"):
        feet_water_bodies.append(area)

func _on_feet_exited_area(area: Area3D):
    if area.is_in_group("water"):
        feet_water_bodies.erase(area)

func _on_head_entered_area(area: Area3D):
    if area.is_in_group("water"):
        head_water_bodies.append(area)

func _on_head_exited_area(area: Area3D):
    if area.is_in_group("water"):
        head_water_bodies.erase(area)
