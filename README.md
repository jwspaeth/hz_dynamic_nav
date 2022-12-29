# hz_dynamic_nav

## Installation

1. **Prepare conda env**
   ```bash
   # We require python>=3.7 and cmake>=3.10
   conda create -n hz_dynamic_nav python=3.7 cmake=3.14.0
   conda activate hz_dynamic_nav
   ```
   
1. **conda install pytorch and cuda**
      ```
      conda install pytorch cudatoolkit=11.3 -c pytorch -c nvidia
      ```

1. **conda install habitat-sim with bullet physics**
   - To install habitat-sim with UI
      ```
      conda install habitat-sim withbullet -c conda-forge -c aihabitat
      ```
      
   - To install habitat-sim headless
      ```
      conda install habitat-sim withbullet headless -c conda-forge -c aihabitat
      ```

1. **pip install habitat-lab**.

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
