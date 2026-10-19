# NUC For Mulinex - Initial Setup

1. Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine).
  You can follow the installation method through `apt`.
  Note that it makes you verify the installation by running `sudo docker run hello-world`.
  It is better to avoid running this command with `sudo` and instead follow the post installation steps first and then run the command without `sudo`.
2. Clone the Docker NUC repository and build the image
   ```shell
   git clone https://github.com/ddebenedittis/docker_nuc_mulinex.git
   cd docker_nuc_mulinex
   ./docker/build.bash
   ```
3. Disable the password on startup.
4. Please, fucking use `cp` instead of `123456` as password, since I do not fucking want to type SIX letters.
5. Enable auto-startup on power on in the bios (a guide is available [here](https://letmegooglethat.com/?q=automatic+turn+on+nuc+power) or [here](https://www.asus.com/support/faq/1052162/)).
6. In the wired settings menu, you can add a new set of IP addresses and netmasks (no gateway) to the same wired connection. This will allow you to concurrently connect the NUC to the personal pc, the Raspberry, and the lidar (or whatever).\
Usually, the personal pc and the raspberry have addresses like `100.100.100.103` and `100.100.100.3`, respectively. The lidar should be `192.168.11.12`. The netmask for all of them is `255.255.255.0`.

## Chrony

1. Install chrony
   ```shell
    sudo apt install chrony
    ```
2. Edit `etc/chrony/chrony.conf` and add the following:
   ```conf
   allow 100.100.100.103/24
   local stratum 10
   ```
3. Restart the service
   ```shell
    sudo systemctl restart chrony.service
    ```

## Final steps

Add this to the `~/.bashrc` of the **normal** user (not root):
```bash
cd mulinex_ws

# Restart chrony
sudo -v
sudo systemctl restart chrony.service
```

# Raspberry Mulinex

## Chrony

Add some lines to the `~/.bashrc` of the **normal** user (not root):
```bash
# Thank you, Attila DB.
# Restart chrony
sudo -v
sudo systemctl restart chrony.service

# Check if chrony is working
check_chrony_100() {
  line="$(chronyc sources | grep -m1 -E '100\.[0-9]+\.[0-9]+\.[0-9]+')"

  if [[ -z "$line" ]]; then
    printf '\e[31m%s\e[0m\n' "No 100.x.x.x source found."
    return
  fi

  marker="${line:0:2}"

  if [[ "$marker" == "^?" ]]; then
    printf '\e[31m%s\e[0m\n' "chronyc needs to be restarted on the nuc"
  else
    printf '\e[32m%s\e[0m\n' "chronyc is ok"
  fi
}

check_chrony_100

sudo su
```

Add these lines to the `~/.bashrc` of the **root** user:
```bash
# Check if chrony is working
check_chrony_100() {
  line="$(chronyc sources | grep -m1 -E '100\.[0-9]+\.[0-9]+\.[0-9]+')"

  if [[ -z "$line" ]]; then
    printf '\e[31m%s\e[0m\n' "No 100.x.x.x source found."
    return
  fi

  marker="${line:0:2}"

  if [[ "$marker" == "^?" ]]; then
    printf '\e[31m%s\e[0m\n' "chronyc needs to be restarted on the nuc"
  else
    printf '\e[32m%s\e[0m\n' "chronyc is ok"
  fi
}

cd new_mulsbc_ws
source /opt/ros/humble/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Now, when you enter `mulsbc`, after entering the password two (2! (not a factorial)) times, it will check if `chrony` is working and print a message accordingly.
If it is ok, you can go ahead.
PD.