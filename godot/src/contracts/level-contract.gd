extends Node3D
class_name LevelBase

# This is the base class for all levels.

@export var level_data: LevelData

func _ready() -> void:
    if not level_data:
        push_error("level data is required")
        get_tree().quit()
        return