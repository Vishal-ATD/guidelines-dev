

# 50 Most Useful Linux Commands

1. **ls**: List files and directories.
   ```bash
   ls
   ```

2. **cd**: Change directory.
   ```bash
   cd /path/to/directory
   ```

3. **pwd**: Print the current working directory.
   ```bash
   pwd
   ```

4. **mkdir**: Create a new directory.
   ```bash
   mkdir directory_name
   ```

5. **rm**: Remove files and directories.
   ```bash
   rm file_name
   ```

6. **cp**: Copy files and directories.
   ```bash
   cp source_file destination_file
   ```

7. **mv**: Move or rename files and directories.
   ```bash
   mv old_name new_name
   ```

8. **cat**: View the contents of a file.
   ```bash
   cat file_name
   ```

9. **head**: Display the first lines of a file.
   ```bash
   head file_name
   ```

10. **tail**: Display the last lines of a file.
    ```bash
    tail file_name
    ```

11. **grep**: Search for a specific pattern in files.
    ```bash
    grep "pattern" file_name
    ```

12. **find**: Search for files and directories.
    ```bash
    find /path/to/search -name "file_name"
    ```

13. **chmod**: Change the permissions of a file or directory.
    ```bash
    chmod permissions file_name
    ```

14. **chown**: Change the owner of a file or directory.
    ```bash
    chown new_owner file_name
    ```

15. **tar**: Compress or extract files.
    ```bash
    tar -cvf archive.tar files   # Compress files
    tar -xvf archive.tar         # Extract files
    ```

16. **gzip**: Compress files.
    ```bash
    gzip file_name
    ```

17. **gunzip**: Decompress files.
    ```bash
    gunzip file_name.gz
    ```

18. **ssh**: Connect to a remote server using SSH.
    ```bash
    ssh username@remote_host
    ```

19. **scp**: Copy files between local and remote machines over SSH.
    ```bash
    scp file_name username@remote_host:/path/to/destination
    ```

20. **wget**: Download files from the web.
    ```bash
    wget URL
    ```

21. **curl**: Transfer data to or from a server.
    ```bash
    curl URL
    ```

22. **ping**: Send ICMP echo requests to a host.
    ```bash
    ping host
    ```

23. **ifconfig**: Display network interface configuration.
    ```bash
    ifconfig
    ```

24. **netstat**: Display network connections and routing tables.
    ```bash
    netstat -a
    ```

25. **ps**: Show running processes.
    ```bash
    ps -ef
    ```

26. **kill**: Terminate a process.
    ```bash
    kill process_id
    ```

27. **top**: Monitor system activity and processes.
    ```bash
    top
    ```

28. **df**: Display disk space usage.
    ```bash
    df -h
    ```

29. **du**: Estimate file and directory disk usage.
    ```bash
    du -sh /path/to/directory
    ```

30. **apt-get**: Package manager for Debian-based systems.
    ```bash
    apt-get install package_name
    ```

31. **yum**: Package manager for RPM-based systems.
    ```bash
    yum install package_name
    ```

32. **history**: View command history.
    ```bash
    history
    ```

33. **man**: Display the manual for a command.
    ```bash
    man command_name
    ```

34. **date**: Display the current date and time.
    ```bash
    date
    ```

35. **cal**: Display a calendar.
    ```bash
    cal
    ```

36. **who**: Display logged-in users.
    ```bash
    who
    ```

37. **useradd**: Add a new user.
    ```bash
    useradd username
    ```

38. **passwd**: Change user password.
    ```bash
    passwd username
    ```

39. **su**: Switch user.
    ```bash
    su username
    ```

40. **sudo**: Execute a command with root privileges.
    ```bash
    sudo command
    ```

41. **shutdown**: Shutdown or restart the system.
    ```bash
    shutdown -h now     # Shutdown immediately
    shutdown -r now     # Restart immediately
    ```

42. **reboot**: Restart the system.
    ```bash
    reboot
    ```

43. **uname**: Display system information.
    ```bash
    uname -a
    ```

44. **lsblk**: List block devices.
    ```bash
    lsblk
    ```

45. **free**: Display memory usage.
    ```bash
    free -h
    ```

46. **scp**: Copy files between local and remote machines over SSH.
    ```bash
    scp file_name username@remote_host:/path/to/destination
    ```

47. **rsync**: Synchronize files and directories between systems.
    ```bash
    rsync -av source/ destination/
    ```

48. **sed**: Stream editor for text manipulation.
    ```bash
    sed 's/old_text/new_text/' file_name
    ```

49. **awk**: Text processing and data extraction tool.
    ```bash
    awk '{ print $1 }' file_name
    ```

50. **echo**: Print text to the console.
    ```bash
    echo "Hello, World!"
    ```

---

Feel free to copy and use this list of commands in Markdown format for your documentation or reference purposes.
