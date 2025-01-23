// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <trace/hooks/sched.h>

#include "walt.h"
#include "trace.h"

unsigned int debugfs_walt_features;
EXPORT_SYMBOL_GPL(debugfs_walt_features);

static struct dentry *debugfs_walt;
void walt_register_debugfs(void)
{
	debugfs_walt = debugfs_create_dir("walt", NULL);
	debugfs_create_u32("walt_features", 0644, debugfs_walt, &debugfs_walt_features);
}
