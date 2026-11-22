# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The TF topology read, the facet's self-disable rule, and what a broadcast puts on the wire. The tf2
lookup is covered in the Jazzy ring."""

from __future__ import annotations

import asyncio

import pytest

from opensmi.ros2.runtime.errors import ROS2RuntimeError
from opensmi.ros2.runtime.tf import (
    TF_STATIC_TOPIC,
    TF_TOPIC,
    FrameLink,
    Transform,
    links,
    roots_of,
)
from tests import fixture_messages
from tests.fake_runtime import FakeRuntime

MESSAGE_TYPES = {
    "tf2_msgs/msg/TFMessage": fixture_messages.TFMessage,
    "fixture_msgs/msg/TransformStamped": fixture_messages.TransformStamped,
    "fixture_msgs/msg/Transform": fixture_messages.Transform,
    "fixture_msgs/msg/Vector3": fixture_messages.Vector3,
    "fixture_msgs/msg/Quaternion": fixture_messages.Quaternion,
    "fixture_msgs/msg/Header": fixture_messages.Header,
    "fixture_msgs/msg/Time": fixture_messages.Time,
}


@pytest.fixture
def runtime() -> FakeRuntime:
    return FakeRuntime(domain_id=7, message_types=MESSAGE_TYPES)


def test_a_link_is_the_header_frame_and_the_child_frame() -> None:
    message = fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom", position=(1.0, 2.0, 3.0)))

    (link,) = links(message, static=False)

    assert link == FrameLink(
        child="odom",
        parent="map",
        transform=Transform(translation=(1.0, 2.0, 3.0), rotation=(0.0, 0.0, 0.0, 1.0)),
        static=False,
    )


def test_a_leading_slash_is_the_same_frame_as_without_one() -> None:
    # tf2 strips the leading slash, so two spellings of one frame must not become two members.
    message = fixture_messages.tf_message(fixture_messages.transform_stamped("/map", "/odom"))

    (link,) = links(message, static=False)

    assert (link.parent, link.child) == ("map", "odom")


@pytest.mark.parametrize(
    ("parent", "child"),
    [("", "odom"), ("map", ""), ("map", "map")],
)
def test_an_element_that_names_no_link_is_skipped(parent, child) -> None:
    message = fixture_messages.tf_message(
        fixture_messages.transform_stamped(parent, child),
        fixture_messages.transform_stamped("map", "odom"),
    )

    assert [link.child for link in links(message, static=False)] == ["odom"]


def test_the_static_flag_travels_with_the_link() -> None:
    message = fixture_messages.tf_message(fixture_messages.transform_stamped("base_link", "lidar"))

    (link,) = links(message, static=True)

    assert link.static is True


def test_the_root_is_the_frame_that_is_nobody_child() -> None:
    frames = {
        link.child: link
        for link in links(
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("map", "odom"),
                fixture_messages.transform_stamped("odom", "base_link"),
            ),
            static=False,
        )
    }

    assert roots_of(frames) == ("map",)


def test_two_trees_published_into_one_graph_have_two_roots() -> None:
    frames = {
        link.child: link
        for link in links(
            fixture_messages.tf_message(
                fixture_messages.transform_stamped("map", "odom"),
                fixture_messages.transform_stamped("world", "cell"),
            ),
            static=False,
        )
    }

    assert roots_of(frames) == ("map", "world")


async def test_the_facet_subscribes_to_nothing_until_something_watches(runtime) -> None:
    await runtime.start()

    assert runtime.tf.reading is False
    assert runtime.tf.frames == {}

    await runtime.stop()


async def test_a_watch_opens_both_topics_and_closing_the_last_one_closes_them(runtime) -> None:
    await runtime.start()

    watch = await runtime.tf.watch()
    assert runtime.tf.reading is True
    assert runtime.subscribed_topics == (TF_TOPIC, TF_STATIC_TOPIC)

    second = await runtime.tf.watch()
    await watch.close()
    assert runtime.tf.reading is True, "one watch closing must not close a topic another is on"

    await second.close()
    assert runtime.tf.reading is False
    assert runtime.subscribed_topics == ()

    await runtime.stop()


async def test_the_frames_are_what_both_topics_have_carried(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()

    runtime.deliver(
        TF_STATIC_TOPIC,
        fixture_messages.tf_message(fixture_messages.transform_stamped("base_link", "lidar")),
    )
    runtime.deliver(TF_TOPIC, fixture_messages.tf_message(fixture_messages.transform_stamped("map", "base_link")))
    await _until(lambda: set(runtime.tf.frames) == {"lidar", "base_link"})

    assert runtime.tf.frames["lidar"].static is True
    assert runtime.tf.frames["base_link"].static is False
    assert runtime.tf.roots == ("map",)

    await watch.close()
    await runtime.stop()


async def test_a_later_message_replaces_a_frame_rather_than_adding_one(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()

    runtime.deliver(TF_TOPIC, fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom")))
    await _until(lambda: "odom" in runtime.tf.frames)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom", position=(9.0, 0.0, 0.0))),
    )
    await _until(lambda: runtime.tf.frames["odom"].transform.translation == (9.0, 0.0, 0.0))

    assert set(runtime.tf.frames) == {"odom"}

    await watch.close()
    await runtime.stop()


async def test_satisfied_is_a_publisher_on_either_topic(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()

    assert runtime.tf.satisfied is False

    runtime.note_publishers(TF_STATIC_TOPIC, 1)
    assert runtime.tf.satisfied is True

    runtime.note_publishers(TF_STATIC_TOPIC, 0)
    assert runtime.tf.satisfied is False

    await watch.close()
    await runtime.stop()


async def test_a_lookup_is_answered_from_the_buffer_or_not_at_all(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()
    composed = Transform(translation=(1.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0))
    runtime.serve_transform(target="map", source="tool0", transform=composed)

    assert runtime.tf.lookup(target="map", source="tool0") == composed
    assert runtime.tf.lookup(target="map", source="nowhere") is None

    await watch.close()
    await runtime.stop()


async def test_stopping_the_runtime_closes_a_watch_nobody_closed(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()

    await runtime.stop()

    assert watch.closed is True
    assert runtime.tf.reading is False


async def _until(ready, within: float = 2.0) -> None:
    """Wait for something the facet's own task does, or fail rather than hang."""
    deadline = asyncio.get_running_loop().time() + within
    while not ready():
        assert asyncio.get_running_loop().time() < deadline, "the facet never got there"
        await asyncio.sleep(0.005)


async def test_two_watches_that_read_tf_from_different_places_are_refused(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch(topic="/robot/tf", static_topic="/robot/tf_static")

    with pytest.raises(ROS2RuntimeError, match="one frame network"):
        await runtime.tf.watch()

    assert runtime.subscribed_topics == ("/robot/tf", "/robot/tf_static")

    await watch.close()
    await runtime.stop()


async def test_a_second_watch_that_agrees_shares_the_one_pair_of_subscriptions(runtime) -> None:
    await runtime.start()
    first = await runtime.tf.watch(topic="/robot/tf", static_topic="/robot/tf_static", depth=5)
    second = await runtime.tf.watch(topic="/robot/tf", static_topic="/robot/tf_static", depth=5)

    assert runtime.subscribed_topics == ("/robot/tf", "/robot/tf_static")

    await first.close()
    assert runtime.tf.reading is True
    await second.close()
    assert runtime.subscribed_topics == ()

    await runtime.stop()


async def test_the_topics_a_remapped_deployment_declares_are_the_ones_read(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch(topic="/robot/tf", static_topic="/robot/tf_static")

    runtime.deliver("/robot/tf", fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom")))
    await _until(lambda: "odom" in runtime.tf.frames)

    await watch.close()
    await runtime.stop()


# --------------------------------------------------------------------------- outward: broadcasting


async def test_nothing_advertises_a_transform_topic_until_a_watch_says_it_publishes(runtime) -> None:
    # A Machine with no frames must not appear as a /tf publisher: other TF Bindings would read it as a source.
    await runtime.start()
    watch = await runtime.tf.watch()

    assert runtime.tf.broadcasting is False
    assert runtime.advertised_topics == ()

    await watch.close()
    await runtime.stop()


async def test_a_watch_that_publishes_advertises_both_topics_from_the_start(runtime) -> None:
    # Discounted from the start: a count that changed meaning at the first tick would flip a required Binding.
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)

    assert runtime.tf.broadcasting is True
    assert runtime.advertised_topics == (TF_TOPIC, TF_STATIC_TOPIC)

    await watch.close()
    assert runtime.advertised_topics == ()

    await runtime.stop()


async def test_a_broadcast_goes_out_as_one_message_naming_each_link_parent_and_child(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)
    runtime.set_clock(11, 12)

    await runtime.tf.broadcast(
        [
            FrameLink(
                child="odom",
                parent="map",
                transform=Transform(translation=(1.0, 2.0, 3.0), rotation=(0.0, 0.0, 0.0, 1.0)),
                static=False,
            )
        ]
    )

    ((topic, message),) = runtime.published
    (element,) = message.transforms
    assert topic == TF_TOPIC
    assert (element.header.frame_id, element.child_frame_id) == ("map", "odom")
    assert (element.transform.translation.x, element.transform.translation.z) == (1.0, 3.0)
    assert element.transform.rotation.w == 1.0

    await watch.close()
    await runtime.stop()


async def test_the_stamp_on_a_broadcast_is_the_runtimes_own_clock(runtime) -> None:
    # Stamped behind the seam: with use_sim_time a stamp taken elsewhere is wrong.
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)
    runtime.set_clock(1_700_000_000, 250)

    await runtime.tf.broadcast([_link("map", "odom")])

    ((_, message),) = runtime.published
    (element,) = message.transforms
    assert (element.header.stamp.sec, element.header.stamp.nanosec) == (1_700_000_000, 250)

    await watch.close()
    await runtime.stop()


async def test_one_broadcast_stamps_every_link_from_one_reading_of_the_clock(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)
    runtime.set_clock(5, 0)

    await runtime.tf.broadcast([_link("map", "odom"), _link("odom", "base_link")])

    ((_, message),) = runtime.published
    assert {(each.header.stamp.sec, each.header.stamp.nanosec) for each in message.transforms} == {(5, 0)}

    await watch.close()
    await runtime.stop()


async def test_a_static_link_goes_to_the_static_topic_and_a_moving_one_does_not(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)

    await runtime.tf.broadcast([_link("map", "odom"), _link("base_link", "lidar", static=True)])

    sent = {topic: [each.child_frame_id for each in message.transforms] for topic, message in runtime.published}
    assert sent == {TF_TOPIC: ["odom"], TF_STATIC_TOPIC: ["lidar"]}

    await watch.close()
    await runtime.stop()


async def test_broadcasting_nothing_puts_no_message_on_either_topic(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)

    await runtime.tf.broadcast([])
    await runtime.tf.broadcast([_link("map", "odom")])

    assert [topic for topic, _ in runtime.published] == [TF_TOPIC]

    await watch.close()
    await runtime.stop()


async def test_broadcasting_without_a_watch_that_publishes_is_refused(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()

    with pytest.raises(ROS2RuntimeError, match="declares no frame of its own"):
        await runtime.tf.broadcast([_link("map", "odom")])

    with pytest.raises(ROS2RuntimeError, match="declares no frame of its own"):
        await runtime.tf.broadcast([])

    await watch.close()
    await runtime.stop()


async def test_our_own_publisher_does_not_make_the_binding_satisfied(runtime) -> None:
    # The matched count includes our own publisher (verified on Jazzy), so we would read ourselves as the robot.
    await runtime.start()
    watch = await runtime.tf.watch(broadcasting=True)

    runtime.note_publishers(TF_TOPIC, 1)
    assert runtime.tf.satisfied is False, "one publisher on /tf is ours"

    runtime.note_publishers(TF_TOPIC, 2)
    assert runtime.tf.satisfied is True, "the second publisher is somebody else's"

    await watch.close()
    await runtime.stop()


def _link(parent: str, child: str, *, static: bool = False) -> FrameLink:
    """Return one link to publish, at the identity pose, which is all these tests need."""
    return FrameLink(
        child=child,
        parent=parent,
        transform=Transform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)),
        static=static,
    )


async def test_a_frame_is_stamped_with_when_it_arrived_and_not_with_its_header(runtime) -> None:
    # The header is publisher time; staleness needs the clock of the one asking.
    await runtime.start()
    watch = await runtime.tf.watch()
    runtime.set_clock(1_700_000_000)

    runtime.deliver(TF_TOPIC, fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom")))
    await _until(lambda: "odom" in runtime.tf.seen)

    assert runtime.tf.seen["odom"] == 1_700_000_000.0
    assert runtime.tf.now == 1_700_000_000.0

    await watch.close()
    await runtime.stop()


async def test_each_frame_is_stamped_on_its_own_rather_than_per_message(runtime) -> None:
    await runtime.start()
    watch = await runtime.tf.watch()

    runtime.set_clock(10)
    runtime.deliver(
        TF_TOPIC,
        fixture_messages.tf_message(
            fixture_messages.transform_stamped("map", "odom"),
            fixture_messages.transform_stamped("odom", "base_link"),
        ),
    )
    await _until(lambda: set(runtime.tf.seen) == {"odom", "base_link"})
    runtime.set_clock(20)
    runtime.deliver(TF_TOPIC, fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom")))
    await _until(lambda: runtime.tf.seen["odom"] == 20.0)

    assert runtime.tf.seen["base_link"] == 10.0, "the frame nobody republished keeps its own age"

    await watch.close()
    await runtime.stop()


async def test_arrivals_taken_before_the_clock_started_are_rebased_when_it_does(runtime) -> None:
    # Simulated time reads zero until the first /clock, then jumps; earlier frames would look ancient.
    await runtime.start()
    watch = await runtime.tf.watch()

    runtime.deliver(TF_TOPIC, fixture_messages.tf_message(fixture_messages.transform_stamped("map", "odom")))
    await _until(lambda: "odom" in runtime.tf.seen)
    assert runtime.tf.seen["odom"] == 0.0, "an unset clock stamps every arrival the same"

    runtime.set_clock(1_700_000_000)
    runtime.deliver(TF_TOPIC, fixture_messages.tf_message(fixture_messages.transform_stamped("map", "base_link")))
    await _until(lambda: "base_link" in runtime.tf.seen)

    assert runtime.tf.seen["odom"] == 1_700_000_000.0, "the frame nothing republished was re-based, not aged"
    assert runtime.tf.seen["base_link"] == 1_700_000_000.0

    await watch.close()
    await runtime.stop()
