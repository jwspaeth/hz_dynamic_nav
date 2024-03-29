from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple, Union

from habitat.core.registry import registry
from habitat.tasks.nav.nav import TopDownMap
from habitat.utils.visualizations import maps


@registry.register_measure
class SocialTopDownMap(TopDownMap):
    def _get_uuid(self, *args: Any, **kwargs: Any) -> str:
        return "social_top_down_map"

    def reset_metric(self, episode, *args: Any, **kwargs: Any):
        super().reset_metric(episode, *args, **kwargs)
        # Draw the paths of the people
        for person_path in episode.people_paths:
            map_corners = [
                maps.to_grid(
                    p[2],
                    p[0],
                    self._top_down_map.shape[0:2],
                    sim=self._sim,
                )
                for p in person_path
            ]
            maps.draw_path(
                self._top_down_map,
                map_corners,
                [255, 165, 0],  # Orange
                self.line_thickness,
            )

    def update_metric(self, episode, action, *args: Any, **kwargs: Any):
        self._step_count += 1
        # Assume only one agent, agent_index=0
        agent_state = self._sim.get_agent_state()
        map_agent_x, map_agent_y = self.update_map(
            agent_state=agent_state, agent_index=0
        )
        agent_map_coords = [[map_agent_x, map_agent_y]]
        agent_angles = [self.get_polar_angle(agent_state)]

        people_map_coords = [
            maps.to_grid(
                p.current_position[2],
                p.current_position[0],
                self._top_down_map.shape[0:2],
                sim=self._sim,
            )
            for p in self._sim.people
        ]

        self._metric = {
            "map": self._top_down_map,
            "fog_of_war_mask": self._fog_of_war_mask,
            "agent_map_coord": agent_map_coords,
            "people_map_coord": people_map_coords,
            "agent_angle": agent_angles,
        }
