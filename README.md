# hz_dynamic_nav

## Installation

1. **Prepare conda env**.
   ```bash
   # We require python>=3.7 and cmake>=3.10
   conda create -n habitat python=3.7 cmake=3.14.0
   conda activate habitat
   ```
   
1. **Install pytorch and cuda**.
      ```
      conda install pytorch cudatoolkit=11.3 -c pytorch -c nvidia
      ```

1. **Install habitat-sim with bullet physics**.
   - To install habitat-sim with UI
      ```
      conda install habitat-sim withbullet -c conda-forge -c aihabitat
      ```
      
   - To install habitat-sim headless
      ```
      conda install habitat-sim withbullet headless -c conda-forge -c aihabitat
      ```

1. **Install habitat-lab**.

      ```bash
      git clone --branch stable https://github.com/facebookresearch/habitat-lab.git
      cd habitat-lab
      git checkout v0.2.3
      pip install -e habitat-lab  # install habitat_lab
      ```
      
1. **Install habitat-baselines**.

    The command above will install only core of Habitat-Lab. To include habitat_baselines along with all additional requirements, use the command below after installing habitat-lab:

      ```bash
      pip install -e habitat-baselines  # install habitat_baselines
      ```
      
1. **Download data**

   Download data as required by habitat-lab and habitat-baselines. This data should be in /hz_dynamic_nav/data. If data is downloaded directly into habitat-lab or habitat-baselines, consider simlinking to /hz_dynamic_nav/data.
   
## Running

1. **Train in hz_dynamic_nav (1 env)**:
   
      ```
      python -um hz_dynamic_nav.run --exp-config hz_dynamic_nav/config/benchmark/nav/dynamicnav/ddppo_dynamicnav.yaml --run-type train habitat_baselines.load_resume_state_config=False habitat_baselines.num_environments=1 habitat_baselines.rl.ppo.num_mini_batch=1
      ```

1. **Eval in hz_dynamic_nav**:
 
      ```
      python -um hz_dynamic_nav.run --exp-config hz_dynamic_nav/config/benchmark/nav/dynamicnav/ddppo_dynamicnav.yaml --run-type eval habitat_baselines.load_resume_state_config=False habitat_baselines.num_environments=1 habitat_baselines.eval_ckpt_path_dir=/coc/testnvme/nyokoyama3/frontier_explorer/slurm/ddppo_pointnav/ckpts/latest.pth habitat_baselines.eval.video_option=["disk"] habitat_baselines.test_episode_count=10
      ```
