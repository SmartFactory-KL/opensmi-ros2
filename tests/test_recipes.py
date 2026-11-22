# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The data recipes without a server or rclpy.

Invertibility is derived from the recipe, and `decode` and `encode` are checked to be inverse where
both exist.
"""

from __future__ import annotations

import struct
from typing import Any

import pytest
from asyncua import ua
from asyncua.ua.object_ids import ObjectIds

from opensmi.ros2.runtime.messages import (
    FieldDescription,
    MessageDescription,
    PrimitiveType,
    build_message_dataclass,
)
from opensmi.ros2.runtime.selection import FieldSelection, resolve_selection
from opensmi.ros2.server.errors import (
    DuplicateProjectionError,
    NotInvertibleError,
    UnprojectableFieldError,
)
from opensmi.ros2.server.events import SIGNED_BYTE_STRING_TYPE
from opensmi.ros2.server.limits import ENVELOPE_MARGIN, byte_string_ceiling
from opensmi.ros2.server.projection import (
    EnumDataType,
    PlanVariable,
    ProjectDataType,
    ProjectionContext,
    ProjectionRegistry,
    StandardDataType,
)
from opensmi.ros2.server.projections import DEFAULT_REGISTRY, SHIPPED
from opensmi.ros2.server.recipes import (
    GENERIC,
    GENERIC_REGISTRY,
    Blob,
    Default,
    Drop,
    Enum,
    Functions,
    Scalar,
)
from tests import fixture_messages


def registry(*entries) -> ProjectionRegistry:
    return GENERIC_REGISTRY.extended(entries)


def context(message_type, *entries, selection=None, qualifier="", ceiling=0) -> ProjectionContext:
    return ProjectionContext(
        registry=registry(*entries),
        selected=resolve_selection(message_type, selection or FieldSelection()),
        qualifier=qualifier,
        byte_string_ceiling=ceiling,
    )


def shape(recipe, message_type, **kwargs):
    return recipe.shape(message_type, context(message_type, recipe, **kwargs))


def decode(recipe, message, **kwargs):
    return recipe.decode(message, context(type(message), recipe, **kwargs))


def encode(recipe, message_type, values, **kwargs):
    return recipe.inverse_of(message_type, values, context(message_type, recipe, **kwargs))


def named(plan) -> dict[str, PlanVariable]:
    return {entry.full_name: entry for entry in plan}


# -- the registry, and what registering means ------------------------------------------------


def test_a_second_entry_for_one_type_is_fatal_unless_it_says_override() -> None:
    curated = registry(Drop("fixture_msgs/msg/Pose", fields=("orientation",)))

    with pytest.raises(DuplicateProjectionError, match="fixture_msgs/msg/Pose"):
        curated.extended([Scalar("fixture_msgs/msg/Pose")])


def test_an_overriding_entry_shadows_the_one_it_replaces() -> None:
    shipped = Drop("fixture_msgs/msg/Pose", fields=("orientation",))
    mine = Drop("fixture_msgs/msg/Pose", fields=("position",), override=True)

    curated = registry(shipped).extended([mine])

    assert curated.find("fixture_msgs/msg/Pose") is mine
    assert curated.entries == (mine, shipped, GENERIC)


def test_registration_is_explicit_and_the_shipped_registry_is_what_it_ships() -> None:
    assert DEFAULT_REGISTRY.entries == (*SHIPPED, GENERIC)


def test_keying_is_by_type_name_and_a_recorded_hash_never_selects() -> None:
    authored = Scalar("fixture_msgs/msg/Voltage", type_hash="RIHS01_0000")
    curated = registry(authored)

    assert curated.find("fixture_msgs/msg/Voltage") is authored
    assert authored.type_hash == "RIHS01_0000"
    assert authored.matches("fixture_msgs/msg/Voltage")


# -- Default, which is also the disable --------------------------------------------------------


def test_the_generic_entry_is_the_default_recipe_answering_for_every_type() -> None:
    assert isinstance(GENERIC, Default)
    assert GENERIC.matches("anything/msg/At/All")


def test_naming_the_generic_entry_for_a_type_disables_curation_at_that_type_only() -> None:
    curated = registry(
        Drop("fixture_msgs/msg/Pose", fields=("orientation",)),
        Drop("fixture_msgs/msg/Point", fields=("z",)),
    )
    disabled = curated.extended([Default("fixture_msgs/msg/Pose", override=True)])

    plan = disabled.find("fixture_msgs/msg/Pose").shape(
        fixture_messages.Pose,
        ProjectionContext(
            registry=disabled,
            selected=resolve_selection(fixture_messages.Pose, FieldSelection()),
        ),
    )

    assert [entry.full_name for entry in plan] == [
        "Position.X",
        "Position.Y",
        "Orientation.X",
        "Orientation.Y",
        "Orientation.Z",
        "Orientation.W",
    ]


# -- Scalar -------------------------------------------------------------------------------------


def test_a_scalar_renders_a_one_value_message_as_the_variable_it_sits_in() -> None:
    recipe = Scalar("fixture_msgs/msg/Voltage")

    (entry,) = shape(recipe, fixture_messages.Voltage)

    assert isinstance(entry, PlanVariable)
    assert entry.name == ""
    assert entry.source == "data"
    assert entry.variant_type == ua.VariantType.Double
    assert decode(recipe, fixture_messages.Voltage(data=23.5)) == {"": 23.5}


def test_a_scalar_may_retype_the_value_it_carries() -> None:
    recipe = Scalar("fixture_msgs/msg/Voltage", variant_type=ua.VariantType.Float)

    (entry,) = shape(recipe, fixture_messages.Voltage)

    assert isinstance(entry, PlanVariable)
    assert entry.variant_type == ua.VariantType.Float


def test_a_scalar_over_a_message_with_a_second_field_is_refused() -> None:
    with pytest.raises(UnprojectableFieldError, match="fixture_msgs/msg/Mode"):
        shape(Scalar("fixture_msgs/msg/Mode"), fixture_messages.Mode)


def test_a_scalar_round_trips() -> None:
    recipe = Scalar("fixture_msgs/msg/Voltage")
    message = fixture_messages.Voltage(data=23.5)

    assert encode(recipe, fixture_messages.Voltage, decode(recipe, message)) == message


# -- Enum ---------------------------------------------------------------------------------------


def test_an_enum_names_its_values_and_leaves_the_rest_of_the_message_alone() -> None:
    recipe = Enum("fixture_msgs/msg/Mode", fields={"mode": {0: "Idle", 7: "Running"}})

    plan = named(shape(recipe, fixture_messages.Mode))

    mode = plan["Mode"]
    assert isinstance(mode, PlanVariable)
    assert mode.variant_type == ua.VariantType.Int32
    assert mode.datatype == EnumDataType(name="Mode_mode", values=((0, "Idle"), (7, "Running")))
    assert isinstance(plan["Detail"], PlanVariable)


def test_an_enum_datatype_name_is_qualified_by_the_component_that_minted_it() -> None:
    recipe = Enum("fixture_msgs/msg/Mode", fields={"mode": {0: "Idle"}})

    left = named(shape(recipe, fixture_messages.Mode, qualifier="left_arm"))["Mode"]
    right = named(shape(recipe, fixture_messages.Mode, qualifier="right_arm"))["Mode"]

    assert isinstance(left, PlanVariable)
    assert isinstance(right, PlanVariable)
    assert isinstance(left.datatype, EnumDataType)
    assert isinstance(right.datatype, EnumDataType)
    assert left.datatype.name == "left_arm_Mode_mode"
    assert right.datatype.name == "right_arm_Mode_mode"


def test_an_enum_keeps_the_ros2_values_rather_than_renumbering_them() -> None:
    recipe = Enum("fixture_msgs/msg/Mode", fields={"mode": {-1: "NoController", 8: "Updating"}})

    mode = named(shape(recipe, fixture_messages.Mode))["Mode"]

    assert isinstance(mode, PlanVariable)
    assert isinstance(mode.datatype, EnumDataType)
    assert mode.datatype.values == ((-1, "NoController"), (8, "Updating"))


def test_an_enum_over_a_field_that_is_not_one_integer_is_refused() -> None:
    with pytest.raises(UnprojectableFieldError, match="detail"):
        shape(Enum("fixture_msgs/msg/Mode", fields={"detail": {0: "Idle"}}), fixture_messages.Mode)


def test_an_enum_carries_the_number_that_was_sent_and_round_trips() -> None:
    recipe = Enum("fixture_msgs/msg/Mode", fields={"mode": {0: "Idle", 7: "Running"}})
    message = fixture_messages.Mode(mode=7, detail="moving")

    values = decode(recipe, message)

    assert values == {"Mode": 7, "Detail": "moving"}
    assert encode(recipe, fixture_messages.Mode, values) == message


# -- Blob ---------------------------------------------------------------------------------------


def test_a_blob_carries_a_byte_array_as_one_byte_string_and_walks_the_rest() -> None:
    recipe = Blob("fixture_msgs/msg/Sensor", field="payload")

    plan = named(shape(recipe, fixture_messages.Sensor))

    payload = plan["Payload"]
    assert isinstance(payload, PlanVariable)
    assert payload.variant_type == ua.VariantType.ByteString
    assert payload.initial == b""
    assert payload.datatype is None
    assert sorted(plan) == ["Labels", "Ok", "Payload", "Readings"]


def test_a_blob_publishes_signedness_as_a_datatype_where_the_elements_are_signed() -> None:
    recipe = Blob("fixture_msgs/msg/Grid", field="data")

    data = named(shape(recipe, fixture_messages.Grid))["Data"]

    assert isinstance(data, PlanVariable)
    assert data.datatype == ProjectDataType(identifier=SIGNED_BYTE_STRING_TYPE)


def test_a_blob_packs_signed_cells_at_their_own_width() -> None:
    recipe = Blob("fixture_msgs/msg/Grid", field="data")

    values = decode(recipe, fixture_messages.Grid(width=2, data=(-1, 0, 100)))

    assert values["Data"] == struct.pack("<3b", -1, 0, 100)
    assert values["Width"] == 2


def test_a_blob_generalises_to_any_fixed_width_primitive_array() -> None:
    recipe = Blob("fixture_msgs/msg/Scan", field="ranges")

    values = decode(recipe, fixture_messages.Scan(frame_id="laser", ranges=(1.5, 2.5)))

    assert values["Ranges"] == struct.pack("<2f", 1.5, 2.5)


def test_a_blob_over_a_field_with_no_fixed_width_is_refused() -> None:
    with pytest.raises(UnprojectableFieldError, match="labels"):
        shape(Blob("fixture_msgs/msg/Sensor", field="labels"), fixture_messages.Sensor)


def test_a_blob_is_capped_at_the_servers_byte_string_ceiling() -> None:
    recipe = Blob("fixture_msgs/msg/Sensor", field="payload")

    values = decode(
        recipe,
        fixture_messages.Sensor(ok=True, labels=(), readings=(), payload=b"0123456789"),
        ceiling=4,
    )

    assert values["Payload"] == b"0123"


def test_a_byte_identity_blob_round_trips() -> None:
    recipe = Blob("fixture_msgs/msg/Grid", field="data")
    message = fixture_messages.Grid(width=3, data=(-1, 2, 3))

    assert encode(recipe, fixture_messages.Grid, decode(recipe, message)) == message


def test_a_blob_that_says_its_bytes_are_something_else_does_not_invert() -> None:
    labelled = Blob("fixture_msgs/msg/Sensor", field="payload", datatype=ObjectIds.ImagePNG)

    assert not labelled.inverts(fixture_messages.Sensor)
    with pytest.raises(NotInvertibleError, match="Blob"):
        encode(labelled, fixture_messages.Sensor, {"Payload": b""})


def test_a_labelled_blob_carries_the_datatype_it_was_given() -> None:
    labelled = Blob("fixture_msgs/msg/Sensor", field="payload", datatype=ObjectIds.ImagePNG)

    payload = named(shape(labelled, fixture_messages.Sensor))["Payload"]

    assert isinstance(payload, PlanVariable)
    assert payload.datatype == StandardDataType(identifier=ObjectIds.ImagePNG)


def test_the_one_explicit_hatch_gives_a_codec_back_its_inverse() -> None:
    def from_png(message_type, values, _context):
        return message_type(ok=True, labels=(), readings=(), payload=b"rebuilt")

    hatched = Blob(
        "fixture_msgs/msg/Sensor",
        field="payload",
        datatype=ObjectIds.ImagePNG,
        encode=from_png,
    )

    assert hatched.inverts(fixture_messages.Sensor)
    rebuilt: Any = encode(hatched, fixture_messages.Sensor, {"Payload": b""})
    assert rebuilt.payload == b"rebuilt"


# -- Drop ---------------------------------------------------------------------------------------


def test_a_drop_removes_a_path_and_everything_under_it() -> None:
    recipe = Drop("fixture_msgs/msg/PoseStamped", fields=("header/stamp", "pose/orientation"))

    plan = shape(recipe, fixture_messages.PoseStamped)

    assert [entry.full_name for entry in plan] == [
        "Header.FrameId",
        "Pose.Position.X",
        "Pose.Position.Y",
        "Pose.Position.Z",
    ]


def test_a_drop_reads_only_what_it_kept() -> None:
    recipe = Drop("fixture_msgs/msg/PoseStamped", fields=("header",))

    values = decode(recipe, fixture_messages.pose_stamped(x=4.0))

    assert set(values) == {"Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z", *_orientation()}
    assert values["Pose.Position.X"] == 4.0


def test_a_drop_never_runs_backwards() -> None:
    # The inverse would fill dropped fields with defaults, and zeros in a covariance do not mean unknown.
    recipe = Drop("fixture_msgs/msg/PoseStamped", fields=("header",))

    assert not recipe.inverts(fixture_messages.PoseStamped)
    with pytest.raises(NotInvertibleError, match="Drop"):
        encode(recipe, fixture_messages.PoseStamped, {})


def test_a_drop_naming_a_path_the_type_does_not_have_is_refused() -> None:
    with pytest.raises(UnprojectableFieldError, match="'pose/covariance'"):
        shape(Drop("fixture_msgs/msg/PoseStamped", fields=("pose/covariance",)), fixture_messages.PoseStamped)


# -- invertibility, as one table ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("recipe", "message_type", "inverts"),
    [
        (Default("fixture_msgs/msg/Point"), fixture_messages.Point, True),
        (Scalar("fixture_msgs/msg/Voltage"), fixture_messages.Voltage, True),
        (Enum("fixture_msgs/msg/Mode", fields={"mode": {0: "Idle"}}), fixture_messages.Mode, True),
        (Blob("fixture_msgs/msg/Grid", field="data"), fixture_messages.Grid, True),
        (Blob("fixture_msgs/msg/Grid", field="data", datatype=ObjectIds.ImagePNG), fixture_messages.Grid, False),
        (Drop("fixture_msgs/msg/Point", fields=("z",)), fixture_messages.Point, False),
        (
            Functions("fixture_msgs/msg/Point", shaped_by=lambda _t, _c: (), decoded_by=lambda _m, _c: {}),
            fixture_messages.Point,
            False,
        ),
    ],
)
def test_invertibility_is_derived_from_the_recipe(recipe, message_type, inverts) -> None:
    assert recipe.inverts(message_type) is inverts


# -- a vendor type, with nothing from the vendor installed ---------------------------------------

ROBOT_MODE = MessageDescription(
    # Shaped like a vendor status message (ten int8 constants, one int8 field); nothing from a vendor is installed.
    type_name="vendor_msgs/msg/RobotMode",
    fields=(FieldDescription(name="mode", field_type=PrimitiveType(name="int8"), ros2_text="int8"),),
)

ROBOT_MODES = {
    -1: "NoController",
    0: "Disconnected",
    1: "ConfirmSafety",
    2: "Booting",
    3: "PowerOff",
    4: "PowerOn",
    5: "Idle",
    6: "Backdrive",
    7: "Running",
    8: "UpdatingFirmware",
}


def test_a_vendor_type_is_curated_and_tested_by_fabricating_its_dataclass() -> None:
    robot_mode: Any = build_message_dataclass(ROBOT_MODE)
    recipe = Enum("vendor_msgs/msg/RobotMode", fields={"mode": ROBOT_MODES})

    plan = named(shape(recipe, robot_mode))
    values = decode(recipe, robot_mode(mode=7))

    mode = plan["Mode"]
    assert isinstance(mode, PlanVariable)
    assert isinstance(mode.datatype, EnumDataType)
    assert mode.datatype.values[0] == (-1, "NoController")
    assert values == {"Mode": 7}


def _orientation() -> set[str]:
    return {f"Pose.Orientation.{axis}" for axis in ("X", "Y", "Z", "W")}


# -- the ceiling a blob is capped at -------------------------------------------------------------


@pytest.mark.parametrize(
    ("budget", "ceiling"),
    [
        (100 * 1024 * 1024, 100 * 1024 * 1024 - ENVELOPE_MARGIN),
        # A budget the envelope allowance would eat whole keeps half; zero means no limit, not that nothing fits.
        (ENVELOPE_MARGIN, ENVELOPE_MARGIN // 2),
        (1_000, 500),
        (1, 1),
        (0, 0),
    ],
)
def test_the_byte_string_ceiling_follows_the_message_budget(budget, ceiling) -> None:
    assert byte_string_ceiling(max_message_size=budget) == ceiling
