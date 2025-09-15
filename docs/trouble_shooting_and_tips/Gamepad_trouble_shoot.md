# Managing the `input` Group on Ubuntu

This guide explains how to check if the `input` group exists, create it if missing, and add a user (e.g., `ubuntu`) to it. Instructions are provided for both outside a Docker container and inside a Docker container.

---

## 🖥️ Outside Docker (host machine)

### 1. Check if `input` group exists
```bash
getent group input
```

- If it prints something like `input:x:107:ubuntu`, the group exists.  
- If nothing prints, the group does not exist.

### 2. Create `input` group if missing
```bash
if ! getent group input > /dev/null; then
    sudo groupadd -g 107 input
fi
```

(`-g 107` is optional — you can omit it and let the system assign the next available GID.)

### 3. Add user `ubuntu` to the group
```bash
sudo usermod -aG input ubuntu
```

### 4. Apply group changes
Either log out and back in, or run:
```bash
newgrp input
```

Check:
```bash
groups ubuntu
```

You should see:
```
ubuntu : ubuntu dialout video input
```

---

## 🐳 Inside Docker (container)

Inside containers, things are a bit different:  
- The default user might be `root`, `ubuntu`, or `vscode` depending on the base image.  
- Group changes usually require editing `/etc/group` and `/etc/passwd` inside the container.  
- You need to commit changes or mount volumes if you want persistence.

### 1. Check if `input` group exists
```bash
getent group input
```

### 2. Create if missing
```bash
if ! getent group input > /dev/null; then
    groupadd -g 107 input
fi
```

(inside Docker, you usually don’t need `sudo` since you’re root already).

### 3. Add user to `input` group
If your user is `ubuntu`:
```bash
usermod -aG input ubuntu
```

If your user is `vscode` (common in dev containers):
```bash
usermod -aG input vscode
```

### 4. Apply group changes
Restart the container or re-login to refresh the group membership.  
Or run:
```bash
newgrp input
```

Check:
```bash
groups ubuntu
```

---

