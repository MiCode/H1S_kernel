#include <linux/version.h>
#include <linux/cred.h>
#include <linux/fs.h>
#include <linux/path.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/printk.h>
#include <linux/namei.h>
#include <linux/list.h>
#include <linux/init_task.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/fdtable.h>
#include <linux/mnt_namespace.h>
#include "internal.h"
#include "mount.h"
#include <linux/susfs.h>

LIST_HEAD(LH_SUS_PATH);
LIST_HEAD(LH_SUS_KSTAT_SPOOFER);
LIST_HEAD(LH_SUS_MOUNT);
LIST_HEAD(LH_SUS_MAPS_SPOOFER);
LIST_HEAD(LH_SUS_PROC_FD_LINK);
LIST_HEAD(LH_SUS_MEMFD);
LIST_HEAD(LH_TRY_UMOUNT_PATH);
LIST_HEAD(LH_MOUNT_ID_RECORDER);

struct st_susfs_uname my_uname;

spinlock_t susfs_spin_lock;
spinlock_t susfs_mnt_id_recorder_spin_lock;

bool is_log_enable = true;
#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
#define SUSFS_LOGI(fmt, ...) if (is_log_enable) pr_info("susfs:[%u][%u][%s] " fmt, current_uid().val, current->pid, __func__, ##__VA_ARGS__)
#define SUSFS_LOGE(fmt, ...) if (is_log_enable) pr_err("susfs:[%u][%u][%s]" fmt, current_uid().val, current->pid, __func__, ##__VA_ARGS__)
#else
#define SUSFS_LOGI(fmt, ...) 
#define SUSFS_LOGE(fmt, ...) 
#endif

int susfs_add_sus_path(struct st_susfs_sus_path* __user user_info) {
	struct st_susfs_sus_path_list *cursor, *temp;
	struct st_susfs_sus_path_list *new_list = NULL;
	struct st_susfs_sus_path info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_path))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH, list) {
		if (unlikely(!strcmp(info.target_pathname, cursor->info.target_pathname))) {
			SUSFS_LOGE("target_pathname: '%s' is already created in LH_SUS_PATH\n", info.target_pathname);
			return 1;
		}
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_path_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_sus_path));

	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_SUS_PATH);
	spin_unlock(&susfs_spin_lock);
	SUSFS_LOGI("target_pathname: '%s' is successfully added to LH_SUS_PATH\n", info.target_pathname);
	return 0;
}

int susfs_add_sus_mount(struct st_susfs_sus_mount* __user user_info) {
	struct st_susfs_sus_mount_list *cursor, *temp;
	struct st_susfs_sus_mount_list *new_list = NULL;
	struct st_susfs_sus_mount info;
	int list_count = 0;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_mount))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MOUNT, list) {
		if (unlikely(!strcmp(cursor->info.target_pathname, info.target_pathname))) {
			SUSFS_LOGE("target_pathname: '%s' is already created in LH_SUS_MOUNT\n", cursor->info.target_pathname);
			return 1;
		}
		list_count++;
	}

	if (list_count == SUSFS_MAX_SUS_MNTS) {
		SUSFS_LOGE("LH_SUS_MOUNT has reached the list limit of %d\n", SUSFS_MAX_SUS_MNTS);
		return 1;
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_mount_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_sus_mount));

	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_SUS_MOUNT);
	spin_unlock(&susfs_spin_lock);
	SUSFS_LOGI("target_pathname: '%s', is successfully added to LH_SUS_MOUNT\n", new_list->info.target_pathname);
	return 0;
}

int susfs_add_sus_kstat(struct st_susfs_sus_kstat* __user user_info) {
	struct st_susfs_sus_kstat_list *cursor, *temp;
	struct st_susfs_sus_kstat_list *new_list = NULL;
	struct st_susfs_sus_kstat info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_kstat))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_KSTAT_SPOOFER, list) {
		if (cursor->info.target_ino == info.target_ino) {
			if (info.target_pathname[0] != '\0') {
				SUSFS_LOGE("target_pathname: '%s' is already created in LH_SUS_KSTAT_SPOOFER\n", info.target_pathname);
			} else {
				SUSFS_LOGE("target_ino: '%lu' is already created in LH_SUS_KSTAT_SPOOFER\n", info.target_ino);
			}
			return 1;
		}
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_kstat_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_sus_kstat));
	/* Seems the dev number issue is finally solved, the userspace stat we see is already a encoded dev
	 * which is set by new_encode_dev() / huge_encode_dev() function for 64bit system and 
	 * old_encode_dev() for 32bit only system, that's why we need to decode it in kernel as well,
	 * and different kernel may have different function to encode the dev number, be cautious!
	 * Also check your encode_dev() macro in fs/stat.c to determine which one to use 
	 */
#if defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64)
#ifdef CONFIG_MIPS
	new_list->info.spoofed_dev = new_decode_dev(new_list->info.spoofed_dev);
#else
	new_list->info.spoofed_dev = huge_decode_dev(new_list->info.spoofed_dev);
#endif /* CONFIG_MIPS */
#else
	new_list->info.spoofed_dev = old_decode_dev(new_list->info.spoofed_dev);
#endif /* defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64) */
	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_SUS_KSTAT_SPOOFER);
	spin_unlock(&susfs_spin_lock);
	SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', spoofed_pathname: '%s', spoofed_ino: '%lu', spoofed_dev: '%lu', spoofed_nlink: '%u', spoofed_atime_tv_sec: '%ld', spoofed_mtime_tv_sec: '%ld', spoofed_ctime_tv_sec: '%ld', spoofed_atime_tv_nsec: '%ld', spoofed_mtime_tv_nsec: '%ld', spoofed_ctime_tv_nsec: '%ld', is successfully added to LH_SUS_KSTAT_SPOOFER\n",
		new_list->info.target_ino , new_list->info.target_pathname, new_list->info.spoofed_pathname,
		new_list->info.spoofed_ino, new_list->info.spoofed_dev, new_list->info.spoofed_nlink,
		new_list->info.spoofed_atime_tv_sec, new_list->info.spoofed_mtime_tv_sec, new_list->info.spoofed_ctime_tv_sec,
		new_list->info.spoofed_atime_tv_nsec, new_list->info.spoofed_mtime_tv_nsec, new_list->info.spoofed_ctime_tv_nsec);
	return 0;
}

int susfs_update_sus_kstat(struct st_susfs_sus_kstat* __user user_info) {
	struct st_susfs_sus_kstat_list *cursor, *temp;
	struct st_susfs_sus_kstat info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_kstat))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_KSTAT_SPOOFER, list) {
		if (unlikely(!strcmp(info.target_pathname, cursor->info.target_pathname))) {
			SUSFS_LOGI("updating target_ino from '%lu' to '%lu' for pathname: '%s' in LH_SUS_KSTAT_SPOOFER\n", cursor->info.target_ino, info.target_ino, info.target_pathname);
			cursor->info.target_ino = info.target_ino;
			return 0;
		}
	}

	SUSFS_LOGE("target_pathname: '%s' is not found in LH_SUS_KSTAT_SPOOFER\n", info.target_pathname);
	return 1;
}

int susfs_add_sus_maps(struct st_susfs_sus_maps* __user user_info) {
	struct st_susfs_sus_maps_list *cursor, *temp;
	struct st_susfs_sus_maps_list *new_list = NULL;
	struct st_susfs_sus_maps info;
	int list_count = 0;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_maps))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

#if defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64)
#ifdef CONFIG_MIPS
	info.target_dev = new_decode_dev(info.target_dev);
#else
	info.target_dev = huge_decode_dev(info.target_dev);
#endif /* CONFIG_MIPS */
#else
	info.target_dev = old_decode_dev(info.target_dev);
#endif /* defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64) */

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MAPS_SPOOFER, list) {
		if (cursor->info.is_statically == info.is_statically && !info.is_statically) {
			if (cursor->info.target_ino == info.target_ino) {
				SUSFS_LOGE("is_statically: '%d', target_ino: '%lu', is already created in LH_SUS_MAPS_SPOOFER\n",
				info.is_statically, info.target_ino);
				return 1;
			}
		} else if (cursor->info.is_statically == info.is_statically && info.is_statically) {
			if (cursor->info.compare_mode == info.compare_mode && info.compare_mode == 1) {
				if (cursor->info.target_ino == info.target_ino) {
					SUSFS_LOGE("is_statically: '%d', compare_mode: '%d', target_ino: '%lu', is already created in LH_SUS_MAPS_SPOOFER\n",
					info.is_statically, info.compare_mode, info.target_ino);
					return 1;
				}
			} else if (cursor->info.compare_mode == info.compare_mode && info.compare_mode == 2) {
				if (cursor->info.target_ino == info.target_ino &&
					cursor->info.is_isolated_entry == info.is_isolated_entry &&
					cursor->info.target_addr_size == info.target_addr_size &&
				    cursor->info.target_pgoff == info.target_pgoff &&
					cursor->info.target_prot == info.target_prot) {
					SUSFS_LOGE("is_statically: '%d', compare_mode: '%d', target_ino: '%lu', is_isolated_entry: '%d', target_pgoff: '0x%x', target_prot: '0x%x', is already created in LH_SUS_MAPS_SPOOFER\n",
					info.is_statically, info.compare_mode, info.target_ino,
					info.is_isolated_entry, info.target_pgoff, info.target_prot);
					return 1;
				}
			} else if (cursor->info.compare_mode == info.compare_mode && info.compare_mode == 3) {
				if (info.target_ino == 0 &&
					cursor->info.prev_target_ino == info.prev_target_ino &&
				    cursor->info.next_target_ino == info.next_target_ino) {
					SUSFS_LOGE("is_statically: '%d', compare_mode: '%d', target_ino: '%lu', prev_target_ino: '%lu', next_target_ino: '%lu', is already created in LH_SUS_MAPS_SPOOFER\n",
					info.is_statically, info.compare_mode, info.target_ino,
					info.prev_target_ino, info.next_target_ino);
					return 1;
				}
			} else if (cursor->info.compare_mode == info.compare_mode && info.compare_mode == 4) {
				if (cursor->info.is_file == info.is_file &&
					cursor->info.target_dev == info.target_dev &&
				    cursor->info.target_pgoff == info.target_pgoff &&
				    cursor->info.target_prot == info.target_prot &&
				    cursor->info.target_addr_size == info.target_addr_size) {
					SUSFS_LOGE("is_statically: '%d', compare_mode: '%d', is_file: '%d', target_dev: '0x%x', target_pgoff: '0x%x', target_prot: '0x%x', target_addr_size: '0x%x', is already created in LH_SUS_MAPS_SPOOFER\n",
					info.is_statically, info.compare_mode, info.is_file,
					info.target_dev, info.target_pgoff, info.target_prot,
					info.target_addr_size);
					return 1;
				}
			}
		}
		list_count++;
	}

	if (list_count == SUSFS_MAX_SUS_MAPS) {
		SUSFS_LOGE("LH_SUS_MOUNT has reached the list limit of %d\n", SUSFS_MAX_SUS_MAPS);
		return 1;
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_maps_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_sus_maps));
#if defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64)
#ifdef CONFIG_MIPS
	new_list->info.spoofed_dev = new_decode_dev(new_list->info.spoofed_dev);
#else
	new_list->info.spoofed_dev = huge_decode_dev(new_list->info.spoofed_dev);
#endif /* CONFIG_MIPS */
#else
	new_list->info.spoofed_dev = old_decode_dev(new_list->info.spoofed_dev);
#endif /* defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64) */
	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_SUS_MAPS_SPOOFER);
	spin_unlock(&susfs_spin_lock);

	SUSFS_LOGI("is_statically: '%d', compare_mode: '%d', is_isolated_entry: '%d', is_file: '%d', prev_target_ino: '%lu', next_target_ino: '%lu', target_ino: '%lu', target_dev: '0x%x', target_pgoff: '0x%x', target_prot: '0x%x', target_addr_size: '0x%x', spoofed_pathname: '%s', spoofed_ino: '%lu', spoofed_dev: '0x%x', spoofed_pgoff: '0x%x', spoofed_prot: '0x%x', is successfully added to LH_SUS_MAPS_SPOOFER\n",
	new_list->info.is_statically, new_list->info.compare_mode, new_list->info.is_isolated_entry,
	new_list->info.is_file, new_list->info.prev_target_ino, new_list->info.next_target_ino,
	new_list->info.target_ino, new_list->info.target_dev, new_list->info.target_pgoff,
	new_list->info.target_prot, new_list->info.target_addr_size, new_list->info.spoofed_pathname,
	new_list->info.spoofed_ino, new_list->info.spoofed_dev, new_list->info.spoofed_pgoff,
	new_list->info.spoofed_prot);

	return 0;
}

int susfs_update_sus_maps(struct st_susfs_sus_maps* __user user_info) {
	struct st_susfs_sus_maps_list *cursor, *temp;
	struct st_susfs_sus_maps info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_maps))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MAPS_SPOOFER, list) {
		if (cursor->info.is_statically == info.is_statically && !info.is_statically) {
			if (unlikely(!strcmp(info.target_pathname, cursor->info.target_pathname))) {
				SUSFS_LOGI("updating target_ino from '%lu' to '%lu' for pathname: '%s' in LH_SUS_MAPS_SPOOFER\n", cursor->info.target_ino, info.target_ino, info.target_pathname);
				cursor->info.target_ino = info.target_ino;
				return 0;
			}
		}
	}

	SUSFS_LOGE("target_pathname: '%s' is not found in LH_SUS_MAPS_SPOOFER\n", info.target_pathname);
	return 1;
}

int susfs_add_sus_proc_fd_link(struct st_susfs_sus_proc_fd_link* __user user_info) {
	struct st_susfs_sus_proc_fd_link_list *cursor, *temp;
	struct st_susfs_sus_proc_fd_link_list *new_list = NULL;
	struct st_susfs_sus_proc_fd_link info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_proc_fd_link))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_PROC_FD_LINK, list) {
		if (unlikely(!strcmp(info.target_link_name, cursor->info.target_link_name))) {
			SUSFS_LOGE("target_link_name: '%s' is already created in LH_SUS_PROC_FD_LINK\n", info.target_link_name);
			return 1;
		}
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_proc_fd_link_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_sus_proc_fd_link));

	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_SUS_PROC_FD_LINK);
	spin_unlock(&susfs_spin_lock);
	SUSFS_LOGI("target_link_name: '%s', spoofed_link_name: '%s', is successfully added to LH_SUS_PROC_FD_LINK\n",
				new_list->info.target_link_name, new_list->info.spoofed_link_name);
	return 0;
}

int susfs_add_sus_memfd(struct st_susfs_sus_memfd* __user user_info) {
	struct st_susfs_sus_memfd_list *cursor, *temp;
	struct st_susfs_sus_memfd_list *new_list = NULL;
	struct st_susfs_sus_memfd info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_sus_memfd))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MEMFD, list) {
		if (unlikely(!strcmp(info.target_pathname, cursor->info.target_pathname))) {
			SUSFS_LOGE("target_pathname: '%s' is already created in LH_SUS_MEMFD\n", info.target_pathname);
			return 1;
		}
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_memfd_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_sus_memfd));

	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_SUS_MEMFD);
	spin_unlock(&susfs_spin_lock);
	SUSFS_LOGI("target_pathname: '%s', is successfully added to LH_SUS_MEMFD\n",
				new_list->info.target_pathname);
	return 0;
}

int susfs_add_try_umount(struct st_susfs_try_umount* __user user_info) {
	struct st_susfs_try_umount_list *cursor, *temp;
	struct st_susfs_try_umount_list *new_list = NULL;
	struct st_susfs_try_umount info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_try_umount))) {
		SUSFS_LOGE("failed copying from userspace\n");
		return 1;
	}

	list_for_each_entry_safe(cursor, temp, &LH_TRY_UMOUNT_PATH, list) {
		if (unlikely(!strcmp(info.target_pathname, cursor->info.target_pathname))) {
			SUSFS_LOGE("target_pathname: '%s' is already created in LH_TRY_UMOUNT_PATH\n", info.target_pathname);
			return 1;
		}
	}

	new_list = kmalloc(sizeof(struct st_susfs_try_umount_list), GFP_KERNEL);
	if (!new_list) {
		SUSFS_LOGE("no enough memory\n");
		return 1;
	}

	memcpy(&new_list->info, &info, sizeof(struct st_susfs_try_umount));

	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock);
	list_add_tail(&new_list->list, &LH_TRY_UMOUNT_PATH);
	spin_unlock(&susfs_spin_lock);
	SUSFS_LOGI("target_pathname: '%s', mnt_mode: %d, is successfully added to LH_TRY_UMOUNT_PATH\n", new_list->info.target_pathname, new_list->info.mnt_mode);
	return 0;
}

int susfs_set_uname(struct st_susfs_uname* __user user_info) {
	struct st_susfs_uname info;

	if (copy_from_user(&info, user_info, sizeof(struct st_susfs_uname))) {
		SUSFS_LOGE("failed copying from userspace.\n");
		return 1;
	}

	spin_lock(&susfs_spin_lock);
	strncpy(my_uname.sysname, info.sysname, __NEW_UTS_LEN);
	strncpy(my_uname.nodename, info.nodename, __NEW_UTS_LEN);
	strncpy(my_uname.release, info.release, __NEW_UTS_LEN);
	strncpy(my_uname.version, info.version, __NEW_UTS_LEN);
	strncpy(my_uname.machine, info.machine, __NEW_UTS_LEN);
	SUSFS_LOGI("setting sysname: '%s', nodename: '%s', release: '%s', version: '%s', machine: '%s'\n",
				my_uname.sysname, my_uname.nodename, my_uname.release, my_uname.version, my_uname.machine);
	spin_unlock(&susfs_spin_lock);
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
int susfs_sus_path_by_path(struct path* file, int* errno_to_be_changed, int syscall_family)
#else
int susfs_sus_path_by_path(const struct path* file, int* errno_to_be_changed, int syscall_family)
#endif
{
	int res = 0;
	int status = 0;
	char* path = NULL;
	char* ptr = NULL;
	char* end = NULL;
	struct st_susfs_sus_path_list *cursor, *temp;

	if (!uid_matches_suspicious_path() || file == NULL) {
		return status;
	}

	path = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (path == NULL) {
		SUSFS_LOGE("no enough memory\n");
		return status;
	}
	ptr = d_path(file, path, PAGE_SIZE);
	if (IS_ERR(ptr)) {
		SUSFS_LOGE("d_path() failed\n");
		goto out_free_path;
	}
	end = mangle_path(path, ptr, " \t\n\\");
	if (!end) {
		goto out_free_path;
	}
	res = end - path;
	path[(size_t) res] = '\0';

	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH, list) {
		if (unlikely(!strcmp(cursor->info.target_pathname, path))) {
			SUSFS_LOGI("hiding target_pathname: '%s', target_ino: '%lu'\n", cursor->info.target_pathname, cursor->info.target_ino);
			if (errno_to_be_changed != NULL) {
				susfs_change_error_no_by_pathname(path, errno_to_be_changed, syscall_family);
			}
			status = 1;
			goto out_free_path;
		}
	}

out_free_path:
	kfree(path);
	return status;
}

int susfs_sus_path_by_filename(struct filename* name, int* errno_to_be_changed, int syscall_family) {
	int status = 0;
	int ret = 0;
	struct path path;

	if (IS_ERR(name)) {
		return status;
	}

	if (!uid_matches_suspicious_path() || name == NULL) {
		return status;
	}

	ret = kern_path(name->name, LOOKUP_FOLLOW, &path);

	if (!ret) {
		status = susfs_sus_path_by_path(&path, errno_to_be_changed, syscall_family);
		path_put(&path);
	}

	return status;
}

int susfs_sus_ino_for_filldir64(unsigned long ino) {
	struct st_susfs_sus_path_list *cursor, *temp;

	if (!uid_matches_suspicious_path())
		return 0;
	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH, list) {
		if (cursor->info.target_ino == ino) {
			SUSFS_LOGI("hiding target_pathname: '%s', target_ino: '%lu'\n", cursor->info.target_pathname, cursor->info.target_ino);
			return 1;
		}
	}
	return 0;
}

int susfs_sus_mount(struct vfsmount* mnt, struct path* root) {
	struct st_susfs_sus_mount_list *cursor, *temp;
	char* path = NULL;
	char* ptr = NULL;
	char* end = NULL;
	int res = 0;
	int status = 0;
	struct path mnt_path = {
		.dentry = mnt->mnt_root,
		.mnt = mnt
	};

	path = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (path == NULL) {
		SUSFS_LOGE("no enough memory\n");
		return 0;
	}
	ptr = __d_path(&mnt_path, root, path, PAGE_SIZE);
	if (IS_ERR(ptr)) {
		SUSFS_LOGE("__d_path() failed\n");
		goto out_free_path;
	}
	end = mangle_path(path, ptr, " \t\n\\");
	if (!end) {
		goto out_free_path;
	}
	res = end - path;
	path[(size_t) res] = '\0';

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MOUNT, list) {
		if (unlikely(!strcmp(path, cursor->info.target_pathname))) {
			SUSFS_LOGI("hide target_pathname '%s' from mounts\n",
						cursor->info.target_pathname);
			status = 1;
			goto out_free_path;
		}
	}
out_free_path:
	kfree(path);
	return status;
}

/*  This function records the original mnt_id and parent_mnt_id of all mounts of
 *  current process and save to a list of corresponding spoofed mnt_id and parent_mnt_id
 *  once process with uid >= 10000 opens /proc/self/mountinfo
 */
void susfs_add_mnt_id_recorder(struct mnt_namespace *ns) {
	struct st_susfs_mnt_id_recorder_list *new_recorder_list = NULL;
	struct st_susfs_mnt_id_recorder_list *recorder_cursor, *recorder_temp;
	struct st_susfs_sus_mount_list *sus_mount_cursor, *sus_mount_temp;
	struct mount *mnt_cursor, *mnt_temp; 
	struct path mnt_path;
	char *path = NULL;
	char *p_path = NULL;
	char *end = NULL;
	int res = 0;
	int cur_pid = current->pid;
	int i = 0, count = 0;

	if (!ns)
		return;

	// if there exists the same pid already, increase the reference
	list_for_each_entry_safe(recorder_cursor, recorder_temp, &LH_MOUNT_ID_RECORDER, list) {
		if (recorder_cursor->pid == cur_pid) {
			recorder_cursor->opened_count++;
			SUSFS_LOGI("mountinfo opened by the same pid: '%d', recorder_cursor->opened_count: '%d'\n",
						cur_pid, recorder_cursor->opened_count);
			return;
		}
	}

	new_recorder_list = kzalloc(sizeof(struct st_susfs_mnt_id_recorder_list), GFP_KERNEL);
	if (!new_recorder_list) {
		SUSFS_LOGE("no enough memory\n");
		return;
	}
	new_recorder_list->info.count = 0;

	path = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!path) {
		SUSFS_LOGE("no enough memory\n");
		goto out_free_new_recorder_list;
	}

	list_for_each_entry_safe(mnt_cursor, mnt_temp, &ns->list, mnt_list) {
		// Avoid overflow
		if (count == SUSFS_MAX_SUS_MNTS) {
			SUSFS_LOGE("LH_MOUNT_ID_RECORDER has reached the list limit of %d\n", SUSFS_MAX_SUS_MNTS);
			goto out_free_path;
		}
		// if this is the first mount entry
		if (count == 0) {
			new_recorder_list->info.target_mnt_id[count] = mnt_cursor->mnt_id;
			new_recorder_list->info.spoofed_mnt_id[count] = mnt_cursor->mnt_id;
			new_recorder_list->info.spoofed_parent_mnt_id[count] = mnt_cursor->mnt_parent->mnt_id;
			new_recorder_list->info.count = ++count;
			continue;
		}

		mntget(&mnt_cursor->mnt);
		dget(mnt_cursor->mnt.mnt_root);
		mnt_path.mnt = &mnt_cursor->mnt;
		mnt_path.dentry = mnt_cursor->mnt.mnt_root;

		p_path = d_path(&mnt_path, path, PAGE_SIZE);
		if (IS_ERR(p_path)) {
			SUSFS_LOGE("d_path() failed\n");
			goto out_continue;
		}
		end = mangle_path(path, p_path, " \t\n\\");
		if (!end) {
			goto out_continue;
		}
		res = end - path;
		path[(size_t) res] = '\0';

		// check if the mount is suspicious
		list_for_each_entry_safe(sus_mount_cursor, sus_mount_temp, &LH_SUS_MOUNT, list) {
			// skip adding this mount to recorder list if it is suspicious
			if (unlikely(!strcmp(path, sus_mount_cursor->info.target_pathname))) {
				SUSFS_LOGI("skip adding target_mnt_id: '%d', target_pathname: '%s' to LH_MOUNT_ID_RECORDER\n",
							mnt_cursor->mnt_id, sus_mount_cursor->info.target_pathname);
				goto out_continue;
			}
		}
		// if the mount entry is NOT suspicioius
		new_recorder_list->info.target_mnt_id[count] = mnt_cursor->mnt_id;
		new_recorder_list->info.spoofed_mnt_id[count] = new_recorder_list->info.spoofed_mnt_id[0] + count;
		for (i = 0; i < count; i++) {
			if (mnt_cursor->mnt_parent->mnt_id == new_recorder_list->info.target_mnt_id[i]) {
				new_recorder_list->info.spoofed_parent_mnt_id[count] = new_recorder_list->info.spoofed_mnt_id[i];
				break;
			}
		}
		// if no match from above, use the original parent mnt_id
		if (new_recorder_list->info.spoofed_parent_mnt_id[count] == 0) {
			new_recorder_list->info.spoofed_parent_mnt_id[count] = mnt_cursor->mnt_parent->mnt_id;
		}
		new_recorder_list->info.count = ++count;
out_continue:
		dput(mnt_cursor->mnt.mnt_root);
		mntput(&mnt_cursor->mnt);
	}

	new_recorder_list->pid = cur_pid;
	new_recorder_list->opened_count = 1;
	kfree(path);

	/*
	for (i = 0; i<new_recorder_list->info.count; i++) {
		SUSFS_LOGI("target_mnt_id: %d, spoofed_mnt_id: %d, spoofed_parent_mnt_id: %d\n",
				new_recorder_list->info.target_mnt_id[i],
				new_recorder_list->info.spoofed_mnt_id[i],
				new_recorder_list->info.spoofed_parent_mnt_id[i]);
	}
	*/

	INIT_LIST_HEAD(&new_recorder_list->list);
	spin_lock(&susfs_mnt_id_recorder_spin_lock);
	list_add_tail(&new_recorder_list->list, &LH_MOUNT_ID_RECORDER);
	spin_unlock(&susfs_mnt_id_recorder_spin_lock);
	SUSFS_LOGI("recording pid '%u' to LH_MOUNT_ID_RECORDER\n", new_recorder_list->pid);
	return;
out_free_path:
	kfree(path);
out_free_new_recorder_list:
	kfree(new_recorder_list);
}

int susfs_get_fake_mnt_id(int mnt_id, int *out_mnt_id, int *out_parent_mnt_id) {
	struct st_susfs_mnt_id_recorder_list *cursor, *temp;
	int cur_pid = current->pid;
	int i;

	list_for_each_entry_safe(cursor, temp, &LH_MOUNT_ID_RECORDER, list) {
		if (cursor->pid == cur_pid) {
			for (i = 0; i < cursor->info.count; i++) {
				if (cursor->info.target_mnt_id[i] == mnt_id) {
					*out_mnt_id = cursor->info.spoofed_mnt_id[i];
					*out_parent_mnt_id = cursor->info.spoofed_parent_mnt_id[i];
					return 0;
				}
			}
			return 1;
		}
	}
	return 1;
}

void susfs_remove_mnt_id_recorder(void) {
	struct st_susfs_mnt_id_recorder_list *cursor, *temp;
	int cur_pid = current->pid;

	spin_lock(&susfs_mnt_id_recorder_spin_lock);
	list_for_each_entry_safe(cursor, temp, &LH_MOUNT_ID_RECORDER, list) {
		if (cursor->pid == cur_pid) {
			cursor->opened_count--;
			if (cursor->opened_count != 0)
				goto out_spin_unlock;
			list_del(&cursor->list);
			kfree(cursor);
			SUSFS_LOGI("removing pid '%u' from LH_MOUNT_ID_RECORDER\n", cur_pid);
			goto out_spin_unlock;
		}
	}
out_spin_unlock:
	spin_unlock(&susfs_mnt_id_recorder_spin_lock);
}

void susfs_sus_kstat(unsigned long ino, struct stat* out_stat) {
	struct st_susfs_sus_kstat_list *cursor, *temp;

	if (!uid_matches_suspicious_kstat())
		return;

	list_for_each_entry_safe(cursor, temp, &LH_SUS_KSTAT_SPOOFER, list) {
		if (cursor->info.target_ino == ino) {
			SUSFS_LOGI("spoofing kstat for pathname '%s' for UID %i\n", cursor->info.target_pathname, current_uid().val);
			out_stat->st_ino = cursor->info.spoofed_ino;
#if defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64)
#ifdef CONFIG_MIPS
			out_stat->st_dev = new_encode_dev(cursor->info.spoofed_dev);
#else
			out_stat->st_dev = huge_encode_dev(cursor->info.spoofed_dev);
#endif /* CONFIG_MIPS */
#else
			out_stat->st_dev = old_encode_dev(cursor->info.spoofed_dev);
#endif /* defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64) */
			out_stat->st_nlink = cursor->info.spoofed_nlink;
			out_stat->st_atime = cursor->info.spoofed_atime_tv_sec;
			out_stat->st_mtime = cursor->info.spoofed_mtime_tv_sec;
			out_stat->st_ctime = cursor->info.spoofed_ctime_tv_sec;
#ifdef _STRUCT_TIMESPEC
			out_stat->st_atime_nsec = cursor->info.spoofed_atime_tv_nsec;
			out_stat->st_mtime_nsec = cursor->info.spoofed_mtime_tv_nsec;
			out_stat->st_ctime_nsec = cursor->info.spoofed_ctime_tv_nsec;
#endif
			return;
		}
	}
}

/* for non statically, it only compare with target_ino, and spoof only the ino, dev to the matched entry
 * for staticially, it compares depending on the mode user chooses
 * compare mode:
 *  1 -> target_ino is 'non-zero', all entries match with target_ino will be spoofed with user defined entry
 *  2 -> target_ino is 'non-zero', all entries match with [target_ino,target_addr_size,target_prot,target_pgoff,is_isolated_entry] will be spoofed with user defined entry
 *  3 -> target_ino is 'zero', which is not file, all entries match with [prev_target_ino,next_target_ino] will be spoofed with user defined entry
 *  4 -> target_ino is 'zero' or 'non-zero', all entries match with [is_file,target_addr_size,target_prot,target_pgoff,target_dev] will be spoofed with user defined entry
 */
int susfs_sus_maps(unsigned long target_ino, unsigned long target_addr_size, unsigned long* orig_ino, dev_t* orig_dev, vm_flags_t* flags, unsigned long long* pgoff, struct vm_area_struct* vma, char* out_name) {
	struct st_susfs_sus_maps_list *cursor, *temp;
	struct inode *tmp_inode, *tmp_inode_prev, *tmp_inode_next;

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MAPS_SPOOFER, list) {
		// if it is NOT statically
		if (!cursor->info.is_statically) {
			if (target_ino != 0 && cursor->info.target_ino == target_ino) {
				*orig_ino = cursor->info.spoofed_ino;
				*orig_dev = cursor->info.spoofed_dev;
				SUSFS_LOGI("spoofing maps -> is_statically: '%d', compare_mode: '%d', is_file: '%d', is_isolated_entry: '%d', prev_target_ino: '%lu', next_target_ino: '%lu', target_ino: '%lu', target_dev: '0x%x', target_pgoff: '0x%x', target_prot: '0x%x', target_addr_size: '0x%x', spoofed_pathname: '%s', spoofed_ino: '%lu', spoofed_dev: '0x%x', spoofed_pgoff: '0x%x', spoofed_prot: '0x%x'\n",
				cursor->info.is_statically, cursor->info.compare_mode, cursor->info.is_file,
				cursor->info.is_isolated_entry, cursor->info.prev_target_ino, cursor->info.next_target_ino,
				cursor->info.target_ino, cursor->info.target_dev, cursor->info.target_pgoff,
				cursor->info.target_prot, cursor->info.target_addr_size, cursor->info.spoofed_pathname,
				cursor->info.spoofed_ino, cursor->info.spoofed_dev, cursor->info.spoofed_pgoff,
				cursor->info.spoofed_prot);
				return 1;
			}
		// if it is statically, then compare with compare_mode
		} else if (cursor->info.compare_mode > 0) {
			switch(cursor->info.compare_mode) {
				case 1:
					if (target_ino != 0 && cursor->info.target_ino == target_ino) {
						goto do_spoof;
					}
					break;
				case 2:
					if (target_ino != 0 && cursor->info.target_ino == target_ino &&
						((cursor->info.target_prot & VM_READ) == (*flags & VM_READ)) &&
						((cursor->info.target_prot & VM_WRITE) == (*flags & VM_WRITE)) &&
						((cursor->info.target_prot & VM_EXEC) == (*flags & VM_EXEC)) &&
						((cursor->info.target_prot & VM_MAYSHARE) == (*flags & VM_MAYSHARE)) &&
						  cursor->info.target_addr_size == target_addr_size &&
						  cursor->info.target_pgoff == *pgoff) {
						// if is NOT isolated_entry, check for vma->vm_next and vma->vm_prev to see if they have the same inode
						if (!cursor->info.is_isolated_entry) {
							if (vma && vma->vm_next) {
								if (vma->vm_next->vm_file) {
									tmp_inode = file_inode(vma->vm_next->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino)
										goto do_spoof;
								}
							}
							if (vma && vma->vm_prev) {
								if (vma->vm_prev->vm_file) {
									tmp_inode = file_inode(vma->vm_prev->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino)
										goto do_spoof;
								}
							}
							continue;
						// if it is isolated_entry
						} else {
							if (vma && vma->vm_next) {
								if (vma->vm_next->vm_file) {
									tmp_inode = file_inode(vma->vm_next->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino) {
										continue;
									}
								}
							}
							if (vma && vma->vm_prev) {
								if (vma->vm_prev->vm_file) {
									tmp_inode = file_inode(vma->vm_prev->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino) {
										continue;
									}
								}
							}
							// both prev and next don't have the same indoe as current entry, we can spoof now
							goto do_spoof;
						}
					}
					break;
				case 3:
					// if current vma is a file, it is not our target
					if (vma->vm_file) continue;
					// compare next target ino only
					if (cursor->info.prev_target_ino == 0 && cursor->info.next_target_ino > 0) {
						if (vma->vm_next && vma->vm_next->vm_file) {
							tmp_inode_next = file_inode(vma->vm_next->vm_file);
							if (tmp_inode_next->i_ino == cursor->info.next_target_ino) {
								goto do_spoof;
							}
						}
					// compare prev target ino only
					} else if (cursor->info.prev_target_ino > 0 && cursor->info.next_target_ino == 0) {
						if (vma->vm_prev && vma->vm_prev->vm_file) {
							tmp_inode_prev = file_inode(vma->vm_prev->vm_file);
							if (tmp_inode_prev->i_ino == cursor->info.prev_target_ino) {
								goto do_spoof;
							}
						}
					// compare both prev ino and next ino
					} else if (cursor->info.prev_target_ino > 0 && cursor->info.next_target_ino > 0) {
						if (vma->vm_prev && vma->vm_prev->vm_file &&
							vma->vm_next && vma->vm_next->vm_file) {
							tmp_inode_prev = file_inode(vma->vm_prev->vm_file);
							tmp_inode_next = file_inode(vma->vm_next->vm_file);
							if (tmp_inode_prev->i_ino == cursor->info.prev_target_ino &&
							    tmp_inode_next->i_ino == cursor->info.next_target_ino) {
								goto do_spoof;
							}
						}
					}
					break;
				case 4:
					if ((cursor->info.is_file && vma->vm_file)||(!cursor->info.is_file && !vma->vm_file)) {
						if (cursor->info.target_dev == *orig_dev &&
							cursor->info.target_pgoff == *pgoff &&
							((cursor->info.target_prot & VM_READ) == (*flags & VM_READ) &&
							 (cursor->info.target_prot & VM_WRITE) == (*flags & VM_WRITE) &&
							 (cursor->info.target_prot & VM_EXEC) == (*flags & VM_EXEC) &&
							 (cursor->info.target_prot & VM_MAYSHARE) == (*flags & VM_MAYSHARE)) &&
							  cursor->info.target_addr_size == target_addr_size) {
							goto do_spoof;
						}
					}
					break;
				default:
					break;
			}
		}
		continue;
do_spoof:
		if (cursor->info.need_to_spoof_pathname) {
			strncpy(out_name, cursor->info.spoofed_pathname, SUSFS_MAX_LEN_PATHNAME-1);
		}
		if (cursor->info.need_to_spoof_ino) {
			*orig_ino = cursor->info.spoofed_ino;
		}
		if (cursor->info.need_to_spoof_dev) {
			*orig_dev = cursor->info.spoofed_dev;
		}
		if (cursor->info.need_to_spoof_prot) {
			if (cursor->info.spoofed_prot & VM_READ) *flags |= VM_READ;
			else *flags = ((*flags | VM_READ) ^ VM_READ);
			if (cursor->info.spoofed_prot & VM_WRITE) *flags |= VM_WRITE;
			else *flags = ((*flags | VM_WRITE) ^ VM_WRITE);
			if (cursor->info.spoofed_prot & VM_EXEC) *flags |= VM_EXEC;
			else *flags = ((*flags | VM_EXEC) ^ VM_EXEC);
			if (cursor->info.spoofed_prot & VM_MAYSHARE) *flags |= VM_MAYSHARE;
			else *flags = ((*flags | VM_MAYSHARE) ^ VM_MAYSHARE);
		}
		if (cursor->info.need_to_spoof_pgoff) {
			*pgoff = cursor->info.spoofed_pgoff;
		}
		SUSFS_LOGI("spoofing maps -> is_statically: '%d', compare_mode: '%d', is_file: '%d', is_isolated_entry: '%d', prev_target_ino: '%lu', next_target_ino: '%lu', target_ino: '%lu', target_dev: '0x%x', target_pgoff: '0x%x', target_prot: '0x%x', target_addr_size: '0x%x', spoofed_pathname: '%s', spoofed_ino: '%lu', spoofed_dev: '0x%x', spoofed_pgoff: '0x%x', spoofed_prot: '0x%x'\n",
		cursor->info.is_statically, cursor->info.compare_mode, cursor->info.is_file,
		cursor->info.is_isolated_entry, cursor->info.prev_target_ino, cursor->info.next_target_ino,
		cursor->info.target_ino, cursor->info.target_dev, cursor->info.target_pgoff,
		cursor->info.target_prot, cursor->info.target_addr_size, cursor->info.spoofed_pathname,
		cursor->info.spoofed_ino, cursor->info.spoofed_dev, cursor->info.spoofed_pgoff,
		cursor->info.spoofed_prot);
		return 2;
	}
	return 0;
}

/* @ This function only does the following:
 *   1. Spoof the symlink name of a target_ino listed in /proc/self/map_files
 * 
 * @Note
 * - It has limitation as there is no way to check which
 *   vma address it belongs by passing dentry* only, so it just
 *   checks for matched dentry* and its target_ino in sus_maps list,
 *   then spoof the symlink name of the target_ino defined by user.
 * - Also user cannot see the effects in map_files from other root session,
 *   because it uses current->mm to compare the dentry, the only way to test
 *   is to check within its own pid.
 * - So the BEST practise here is:
 *     Do NOT spoof the map entries which share the same name to different name
 *     seperately unless the other spoofed name is empty of which spoofed_ino is 0,
 *     otherwise there will be inconsistent entries between maps and map_files.
 */
void susfs_sus_map_files_readlink(unsigned long target_ino, char* pathname) {
	struct st_susfs_sus_maps_list *cursor, *temp;

	if (!pathname)
		return;

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MAPS_SPOOFER, list) {
		// We are only interested in statically and target_ino > 0
		if (cursor->info.is_statically && cursor->info.compare_mode > 0 &&
			target_ino > 0 && cursor->info.target_ino == target_ino)
		{
			if (cursor->info.need_to_spoof_pathname) {
				SUSFS_LOGI("spoofing symlink name of ino '%lu' to '%s' in map_files\n",
						target_ino, cursor->info.spoofed_pathname);
				// Don't need to check buffer size as 'pathname' is allocated with 'PAGE_SIZE'
				// which is way bigger than SUSFS_MAX_LEN_PATHNAME
				strcpy(pathname, cursor->info.spoofed_pathname);
				return;
			}
		}
	}
	return;
}

/* @ This function mainly does the following:
 *   1. Remove the user write access for spoofed symlink name in /proc/self/map_files
 *   2. Prevent the dentry from being seen in /proc/self/map_files
 * 
 * @Note
 * - anon files are supposed to be not shown in /proc/self/map_files and 
 *   spoofing from memfd name to non-memfd name should not have write
 *   permission on that target dentry
 */
int susfs_sus_map_files_instantiate(struct vm_area_struct* vma) {
	struct inode *inode = file_inode(vma->vm_file);
	unsigned long target_ino = inode->i_ino;
	dev_t target_dev = inode->i_sb->s_dev;
	unsigned long long target_pgoff = ((loff_t)vma->vm_pgoff) << PAGE_SHIFT;
	unsigned long target_addr_size = vma->vm_end - vma->vm_start;
	vm_flags_t target_flags = vma->vm_flags;
	struct st_susfs_sus_maps_list *cursor, *temp;
	struct inode *tmp_inode, *tmp_inode_prev, *tmp_inode_next;

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MAPS_SPOOFER, list) {
		// We are only interested in statically
		if (!cursor->info.is_statically) {
			continue;
		// if it is statically, then compare with compare_mode
		} else if (cursor->info.compare_mode > 0) {
			switch(cursor->info.compare_mode) {
				case 1:
					if (target_ino != 0 && cursor->info.target_ino == target_ino) {
						goto do_spoof;
					}
					break;
				case 2:
					if (target_ino != 0 && cursor->info.target_ino == target_ino &&
						((cursor->info.target_prot & VM_READ) == (target_flags & VM_READ)) &&
						((cursor->info.target_prot & VM_WRITE) == (target_flags & VM_WRITE)) &&
						((cursor->info.target_prot & VM_EXEC) == (target_flags & VM_EXEC)) &&
						((cursor->info.target_prot & VM_MAYSHARE) == (target_flags & VM_MAYSHARE)) &&
						  cursor->info.target_addr_size == target_addr_size &&
						  cursor->info.target_pgoff == target_pgoff) {
						// if is NOT isolated_entry, check for vma->vm_next and vma->vm_prev to see if they have the same inode
						if (!cursor->info.is_isolated_entry) {
							if (vma && vma->vm_next) {
								if (vma->vm_next->vm_file) {
									tmp_inode = file_inode(vma->vm_next->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino)
										goto do_spoof;
								}
							}
							if (vma && vma->vm_prev) {
								if (vma->vm_prev->vm_file) {
									tmp_inode = file_inode(vma->vm_prev->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino)
										goto do_spoof;
								}
							}
							continue;
						// if it is isolated_entry
						} else {
							if (vma && vma->vm_next) {
								if (vma->vm_next->vm_file) {
									tmp_inode = file_inode(vma->vm_next->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino) {
										continue;
									}
								}
							}
							if (vma && vma->vm_prev) {
								if (vma->vm_prev->vm_file) {
									tmp_inode = file_inode(vma->vm_prev->vm_file);
									if (tmp_inode->i_ino == cursor->info.target_ino) {
										continue;
									}
								}
							}
							// both prev and next don't have the same indoe as current entry, we can spoof now
							goto do_spoof;
						}
					}
					break;
				case 3:
					// if current vma is a file, it is not our target
					if (vma->vm_file) continue;
					// compare next target ino only
					if (cursor->info.prev_target_ino == 0 && cursor->info.next_target_ino > 0) {
						if (vma->vm_next && vma->vm_next->vm_file) {
							tmp_inode_next = file_inode(vma->vm_next->vm_file);
							if (tmp_inode_next->i_ino == cursor->info.next_target_ino) {
								goto do_spoof;
							}
						}
					// compare prev target ino only
					} else if (cursor->info.prev_target_ino > 0 && cursor->info.next_target_ino == 0) {
						if (vma->vm_prev && vma->vm_prev->vm_file) {
							tmp_inode_prev = file_inode(vma->vm_prev->vm_file);
							if (tmp_inode_prev->i_ino == cursor->info.prev_target_ino) {
								goto do_spoof;
							}
						}
					// compare both prev ino and next ino
					} else if (cursor->info.prev_target_ino > 0 && cursor->info.next_target_ino > 0) {
						if (vma->vm_prev && vma->vm_prev->vm_file &&
							vma->vm_next && vma->vm_next->vm_file) {
							tmp_inode_prev = file_inode(vma->vm_prev->vm_file);
							tmp_inode_next = file_inode(vma->vm_next->vm_file);
							if (tmp_inode_prev->i_ino == cursor->info.prev_target_ino &&
							    tmp_inode_next->i_ino == cursor->info.next_target_ino) {
								goto do_spoof;
							}
						}
					}
					break;
				case 4:
					if ((cursor->info.is_file && vma->vm_file)||(!cursor->info.is_file && !vma->vm_file)) {
						if (cursor->info.target_dev == target_dev &&
							cursor->info.target_pgoff == target_pgoff &&
							((cursor->info.target_prot & VM_READ) == (target_flags & VM_READ) &&
							 (cursor->info.target_prot & VM_WRITE) == (target_flags & VM_WRITE) &&
							 (cursor->info.target_prot & VM_EXEC) == (target_flags & VM_EXEC) &&
							 (cursor->info.target_prot & VM_MAYSHARE) == (target_flags & VM_MAYSHARE)) &&
							  cursor->info.target_addr_size == target_addr_size) {
							goto do_spoof;
						}
					}
					break;
				default:
					break;
			}
		}
		continue;
do_spoof:
		if (!(cursor->info.spoofed_ino == 0 ||
			(MAJOR(cursor->info.spoofed_dev) == 0 &&
			(MINOR(cursor->info.spoofed_dev) == 0 || MINOR(cursor->info.spoofed_dev) == 1))))
		{
			SUSFS_LOGI("remove user write permission of spoofed symlink '%s' in map_files\n", cursor->info.spoofed_pathname);
			return 1;
		} else {
			SUSFS_LOGI("drop dentry of target_ino '%lu' with spoofed_ino '%lu' in map_files\n",
						cursor->info.target_ino, cursor->info.spoofed_ino);
			return 2;
		}
		return 0;
	}
	return 0;
}

int susfs_is_sus_maps_list_empty(void) {
	return list_empty(&LH_SUS_MAPS_SPOOFER);
}

int susfs_sus_proc_fd_link(char *pathname, int len) {
	struct st_susfs_sus_proc_fd_link_list *cursor, *temp;

	list_for_each_entry_safe(cursor, temp, &LH_SUS_PROC_FD_LINK, list) {
		if (unlikely(!strcmp(pathname, cursor->info.target_link_name))) {
			SUSFS_LOGI("[uid:%u] spoofing fd link: '%s' -> '%s'\n", current_uid().val, pathname, cursor->info.spoofed_link_name);
			memset(pathname, 0, len);
			strcpy(pathname, cursor->info.spoofed_link_name);
			return 1;
		}
	}
	return 0;
}

int susfs_is_sus_proc_fd_link_list_empty(void) {
	return list_empty(&LH_SUS_PROC_FD_LINK);
}

int susfs_sus_memfd(char *memfd_name) {
	struct st_susfs_sus_memfd_list *cursor, *temp;

	list_for_each_entry_safe(cursor, temp, &LH_SUS_MEMFD, list) {
		if (unlikely(!strcmp(memfd_name, cursor->info.target_pathname))) {
			SUSFS_LOGI("prevent memfd_name: '%s' from being created\n", memfd_name);
			return 1;
		}
	}
    return 0;
}

static void umount_mnt(struct path *path, int flags) {
	int err = path_umount(path, flags);
	if (err) {
		SUSFS_LOGI("umount %s failed: %d\n", path->dentry->d_iname, err);
	}
}

static bool should_umount(struct path *path)
{
	if (!path) {
		return false;
	}

	if (current->nsproxy->mnt_ns == init_nsproxy.mnt_ns) {
		SUSFS_LOGI("ignore global mnt namespace process: %d\n",
			current_uid().val);
		return false;
	}

	if (path->mnt && path->mnt->mnt_sb && path->mnt->mnt_sb->s_type) {
		const char *fstype = path->mnt->mnt_sb->s_type->name;
		return strcmp(fstype, "overlay") == 0;
	}
	return false;
}

static void try_umount(const char *mnt, bool check_mnt, int flags) {
	struct path path;
	int err = kern_path(mnt, 0, &path);

	if (err) {
		return;
	}

	if (path.dentry != path.mnt->mnt_root) {
		// it is not root mountpoint, maybe umounted by others already.
		return;
	}

	// we are only interest in some specific mounts
	if (check_mnt && !should_umount(&path)) {
		return;
	}
	
	umount_mnt(&path, flags);
}

void susfs_try_umount(uid_t target_uid) {
	struct st_susfs_try_umount_list *cursor, *temp;

	list_for_each_entry_safe(cursor, temp, &LH_TRY_UMOUNT_PATH, list) {
		SUSFS_LOGI("umounting '%s' for uid: %d\n", cursor->info.target_pathname, target_uid);
		if (cursor->info.mnt_mode == 0) {
			try_umount(cursor->info.target_pathname, false, 0);
		} else if (cursor->info.mnt_mode == 1) {
			try_umount(cursor->info.target_pathname, false, MNT_DETACH);
		}
	}
}

void susfs_spoof_uname(struct new_utsname* tmp) {
	if (strcmp(my_uname.sysname, "default")) {
		memset(tmp->sysname, 0, __NEW_UTS_LEN);
		strncpy(tmp->sysname, my_uname.sysname, __NEW_UTS_LEN);
	}
	if (strcmp(my_uname.nodename, "default")) {
		memset(tmp->nodename, 0, __NEW_UTS_LEN);
		strncpy(tmp->nodename, my_uname.nodename, __NEW_UTS_LEN);
	}
	if (likely(strcmp(my_uname.release, "default"))) {
		memset(tmp->release, 0, __NEW_UTS_LEN);
		strncpy(tmp->release, my_uname.release, __NEW_UTS_LEN);
	}
	if (likely(strcmp(my_uname.version, "default"))) {
		memset(tmp->version, 0, __NEW_UTS_LEN);
		strncpy(tmp->version, my_uname.version, __NEW_UTS_LEN);
	}
	if (strcmp(my_uname.machine, "default")) {
		memset(tmp->machine, 0, __NEW_UTS_LEN);
		strncpy(tmp->machine, my_uname.machine, __NEW_UTS_LEN);
	}
}

void susfs_set_log(bool enabled) {
	spin_lock(&susfs_spin_lock);
	is_log_enable = enabled;
	spin_unlock(&susfs_spin_lock);
	if (is_log_enable) {
		pr_info("susfs: enable logging to kernel");
	} else {
		pr_info("susfs: disable logging to kernel");
	}
}

/* For files/directories in /sdcard/ but not in /sdcard/Android/data/, please delete it  
 * by yourself
 */
void susfs_change_error_no_by_pathname(char* const pathname, int* const errno_to_be_changed, int const syscall_family) {
	if (!strncmp(pathname, "/system/", 8)||
		!strncmp(pathname, "/vendor/", 8)) {
		switch(syscall_family) {
			case SYSCALL_FAMILY_ALL_ENOENT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_LINKAT_OLDNAME:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_OLDNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			//case SYSCALL_FAMILY_RENAMEAT2_NEWNAME:
			//	if (!strncmp(pathname, "/system/", 8)) {
			//		*errno_to_be_changed = -EROFS;
			//	} else {
			//		*errno_to_be_changed = -EXDEV;
			//	}
			//	return;
			default:
				*errno_to_be_changed = -EROFS;
				return;
		}
	} else if (!strncmp(pathname, "/storage/emulated/0/Android/data/", 33)) {
		switch(syscall_family) {
			case SYSCALL_FAMILY_ALL_ENOENT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_MKNOD:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_MKDIRAT:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_RMDIR:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_UNLINKAT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_SYMLINKAT_NEWNAME:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_LINKAT_OLDNAME:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_LINKAT_NEWNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_OLDNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_NEWNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			default:
				*errno_to_be_changed = -ENOENT;
				return;
		}
	} else if (!strncmp(pathname, "/dev/", 5)) {
		switch(syscall_family) {
			case SYSCALL_FAMILY_ALL_ENOENT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_MKNOD:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_MKDIRAT:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_RMDIR:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_UNLINKAT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_SYMLINKAT_NEWNAME:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_LINKAT_OLDNAME:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_LINKAT_NEWNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_OLDNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_NEWNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			default:
				*errno_to_be_changed = -ENOENT;
				return;
		}
	} else if (!strncmp(pathname, "/data/", 6)) {
				switch(syscall_family) {
			case SYSCALL_FAMILY_ALL_ENOENT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_MKNOD:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_MKDIRAT:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_RMDIR:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_UNLINKAT:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_SYMLINKAT_NEWNAME:
				*errno_to_be_changed = -EACCES;
				return;
			case SYSCALL_FAMILY_LINKAT_OLDNAME:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_LINKAT_NEWNAME:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_OLDNAME:
				*errno_to_be_changed = -ENOENT;
				return;
			case SYSCALL_FAMILY_RENAMEAT2_NEWNAME:
				*errno_to_be_changed = -EXDEV;
				return;
			default:
				*errno_to_be_changed = -ENOENT;
				return;
		}
	}
}

/*
static int susfs_get_cur_fd_counts() {
	struct fdtable *files_table;
    int fd_count = 0;

	files_table = files_fdtable(current->files);
	for (i = 0; i < files_table->max_fds; i++) {
        if (files_table->fd[i] != NULL) {
            fd_count++;
        }
    }
	return fd_count;
}
*/

static void susfs_my_uname_init(void) {
	memset(&my_uname, 0, sizeof(struct st_susfs_uname));
	strncpy(my_uname.sysname, "default", __NEW_UTS_LEN);
	strncpy(my_uname.nodename, "default", __NEW_UTS_LEN);
	strncpy(my_uname.release, "default", __NEW_UTS_LEN);
	strncpy(my_uname.version, "default", __NEW_UTS_LEN);
	strncpy(my_uname.machine, "default", __NEW_UTS_LEN);
}

void __init susfs_init(void) {
	spin_lock_init(&susfs_spin_lock);
	spin_lock_init(&susfs_mnt_id_recorder_spin_lock);
	susfs_my_uname_init();
}

/* No module exit is needed becuase it should never be a loadable kernel module */
//void __init susfs_exit(void)
