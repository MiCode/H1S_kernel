#ifndef KSU_SUSFS_H
#define KSU_SUSFS_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/utsname.h>
#include <linux/mount.h>

/* shared with userspace ksu_susfs tool */
#define CMD_SUSFS_ADD_SUS_PATH 0x55555
#define CMD_SUSFS_ADD_SUS_MOUNT 0x55556
#define CMD_SUSFS_ADD_SUS_KSTAT 0x55558
#define CMD_SUSFS_UPDATE_SUS_KSTAT 0x55559
#define CMD_SUSFS_ADD_TRY_UMOUNT 0x5555a
#define CMD_SUSFS_SET_UNAME 0x5555b
#define CMD_SUSFS_ADD_SUS_KSTAT_STATICALLY 0x5555c
#define CMD_SUSFS_ENABLE_LOG 0x5555d
#define CMD_SUSFS_ADD_SUS_MAPS_STATICALLY 0x5555e
#define CMD_SUSFS_ADD_SUS_PROC_FD_LINK 0x5555f
#define CMD_SUSFS_ADD_SUS_MAPS 0x55560
#define CMD_SUSFS_UPDATE_SUS_MAPS 0x55561
#define CMD_SUSFS_ADD_SUS_MEMFD 0x55562

#define SUSFS_MAX_LEN_PATHNAME 256 // 256 should address many paths already unless you are doing some strange experimental stuff, then set your own desired length
#define SUSFS_MAX_LEN_MFD_NAME 248
#define SUSFS_MAX_SUS_MNTS 300 // I think 300 is now enough? This includes the mount entries for each process and sus mounts added by user 
#define SUSFS_MAX_SUS_MAPS 200 // I think 200 is now enough? Tell me why if you have over 200 entries

#define SUSFS_MAP_FILES_ACTION_REMOVE_WRITE_PERM 1
#define SUSFS_MAP_FILES_ACTION_HIDE_DENTRY 2

/* non shared to userspace ksu_susfs tool */
#define SYSCALL_FAMILY_ALL_ENOENT 0
#define SYSCALL_FAMILY_OPENAT 1
#define SYSCALL_FAMILY_MKNOD 2
#define SYSCALL_FAMILY_MKDIRAT 3
#define SYSCALL_FAMILY_RMDIR 4
#define SYSCALL_FAMILY_UNLINKAT 5
#define SYSCALL_FAMILY_SYMLINKAT_NEWNAME 6
#define SYSCALL_FAMILY_LINKAT_OLDNAME 7
#define SYSCALL_FAMILY_LINKAT_NEWNAME 8
#define SYSCALL_FAMILY_RENAMEAT2_OLDNAME 9
#define SYSCALL_FAMILY_RENAMEAT2_NEWNAME 10
#define SYSCALL_FAMILY_TRUNCATE 11
#define SYSCALL_FAMILY_FACCESSAT 12
#define SYSCALL_FAMILY_CHDIR 13

#define getname_safe(name) (name == NULL ? ERR_PTR(-EINVAL) : getname(name))
#define putname_safe(name) (IS_ERR(name) ? NULL : putname(name))

#define uid_matches_suspicious_path() (current_uid().val >= 2000)
#define uid_matches_suspicious_kstat() (current_uid().val >= 2000)
#define uid_matches_proc_need_to_reorder_mnt_id() (current_uid().val >= 10000)

struct st_susfs_sus_path {
	char                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	unsigned long           target_ino;
};

struct st_susfs_sus_mount {
	char                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
};

struct st_susfs_sus_kstat {
	unsigned long           target_ino; // the ino after bind mounted or overlayed
	char                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	char                    spoofed_pathname[SUSFS_MAX_LEN_PATHNAME];
	unsigned long           spoofed_ino;
	unsigned long           spoofed_dev;
	unsigned int            spoofed_nlink;
	long                    spoofed_atime_tv_sec;
	long                    spoofed_mtime_tv_sec;
	long                    spoofed_ctime_tv_sec;
	long                    spoofed_atime_tv_nsec;
	long                    spoofed_mtime_tv_nsec;
	long                    spoofed_ctime_tv_nsec;
};

struct st_susfs_sus_maps {
	bool                    is_statically;
	int                     compare_mode;
	bool                    is_isolated_entry;
	bool                    is_file;
	unsigned long           prev_target_ino;
	unsigned long           next_target_ino;
	char                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	unsigned long           target_ino;
	unsigned long           target_dev;
	unsigned long long      target_pgoff;
	unsigned long           target_prot;
	unsigned long           target_addr_size;
	char                    spoofed_pathname[SUSFS_MAX_LEN_PATHNAME];
	unsigned long           spoofed_ino;
	unsigned long           spoofed_dev;
	unsigned long long      spoofed_pgoff;
	unsigned long           spoofed_prot;
	bool                    need_to_spoof_pathname;
	bool                    need_to_spoof_ino;
	bool                    need_to_spoof_dev;
	bool                    need_to_spoof_pgoff;
	bool                    need_to_spoof_prot;
};

struct st_susfs_try_umount {
	char                    target_pathname[SUSFS_MAX_LEN_PATHNAME];
	int                     mnt_mode;
};

struct st_susfs_sus_proc_fd_link {
	char                    target_link_name[SUSFS_MAX_LEN_PATHNAME];
	char                    spoofed_link_name[SUSFS_MAX_LEN_PATHNAME];
};

struct st_susfs_sus_memfd {
	char                    target_pathname[SUSFS_MAX_LEN_MFD_NAME];
};

struct st_susfs_mnt_id_recorder {
	int                     target_mnt_id[SUSFS_MAX_SUS_MNTS];
	int                     spoofed_mnt_id[SUSFS_MAX_SUS_MNTS];
	int                     spoofed_parent_mnt_id[SUSFS_MAX_SUS_MNTS];
	int                     count;
};

struct st_susfs_sus_path_list {
	struct list_head                        list;
	struct st_susfs_sus_path                info;
};

struct st_susfs_sus_mount_list {
	struct list_head                        list;
	struct st_susfs_sus_mount               info;
};

struct st_susfs_sus_kstat_list {
	struct list_head                        list;
	struct st_susfs_sus_kstat               info;
};

struct st_susfs_sus_maps_list {
	struct list_head                        list;
	struct st_susfs_sus_maps                info;
};

struct st_susfs_try_umount_list {
	struct list_head                        list;
	struct st_susfs_try_umount              info;
};

struct st_susfs_sus_proc_fd_link_list {
	struct list_head                        list;
	struct st_susfs_sus_proc_fd_link        info;
};

struct st_susfs_sus_memfd_list {
	struct list_head                        list;
	struct st_susfs_sus_memfd               info;
};

struct st_susfs_mnt_id_recorder_list {
	struct list_head                        list;
	int                                     pid;
	int                                     opened_count;
	struct st_susfs_mnt_id_recorder         info;
};

struct st_susfs_uname {
	char        sysname[__NEW_UTS_LEN+1];
	char        nodename[__NEW_UTS_LEN+1];
	char        release[__NEW_UTS_LEN+1];
	char        version[__NEW_UTS_LEN+1];
	char        machine[__NEW_UTS_LEN+1];
};

int susfs_add_sus_path(struct st_susfs_sus_path* __user user_info);
int susfs_add_sus_mount(struct st_susfs_sus_mount* __user user_info);
int susfs_add_sus_kstat(struct st_susfs_sus_kstat* __user user_info);
int susfs_update_sus_kstat(struct st_susfs_sus_kstat* __user user_info);
int susfs_add_sus_maps(struct st_susfs_sus_maps* __user user_info);
int susfs_update_sus_maps(struct st_susfs_sus_maps* __user user_info);
int susfs_add_sus_proc_fd_link(struct st_susfs_sus_proc_fd_link* __user user_info);
int susfs_add_sus_memfd(struct st_susfs_sus_memfd* __user user_info);
int susfs_add_try_umount(struct st_susfs_try_umount* __user user_info);
int susfs_set_uname(struct st_susfs_uname* __user user_info);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
int susfs_sus_path_by_path(struct path* file, int* errno_to_be_changed, int syscall_family);
#else
int susfs_sus_path_by_path(const struct path* file, int* errno_to_be_changed, int syscall_family);
#endif
int susfs_sus_path_by_filename(struct filename* name, int* errno_to_be_changed, int syscall_family);
int susfs_sus_mount(struct vfsmount* mnt, struct path* root);
int susfs_sus_ino_for_filldir64(unsigned long ino);
void susfs_sus_kstat(unsigned long ino, struct stat* out_stat);
int susfs_sus_maps(unsigned long target_ino, unsigned long target_addr_size,
					unsigned long* orig_ino, dev_t* orig_dev, vm_flags_t* flags,
					unsigned long long* pgoff, struct vm_area_struct* vma, char* out_name);
void susfs_sus_map_files_readlink(unsigned long target_ino, char* pathname);
int susfs_sus_map_files_instantiate(struct vm_area_struct* vma);
int susfs_is_sus_maps_list_empty(void);
int susfs_sus_proc_fd_link(char *pathname, int len);
int susfs_is_sus_proc_fd_link_list_empty(void);
int susfs_sus_memfd(char *memfd_name);
void susfs_try_umount(uid_t target_uid);
void susfs_spoof_uname(struct new_utsname* tmp);
void susfs_add_mnt_id_recorder(struct mnt_namespace *ns);
int susfs_get_fake_mnt_id(int mnt_id, int *out_mnt_id, int *out_parent_mnt_id);
void susfs_remove_mnt_id_recorder(void);

void susfs_set_log(bool enabled);

void susfs_change_error_no_by_pathname(char* pathname, int* errno_to_be_changed, int syscall_family);

void __init susfs_init(void);

#endif
