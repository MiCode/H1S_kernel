/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*! \file   "ie_sort.h"
 *  \brief
 */

#ifndef _IE_SORT_H
#define _IE_SORT_H

void sortAssocReqIE(IN struct ADAPTER *prAdapter,
			IN struct MSDU_INFO *prMsduInfo,
			IN uint8_t fgIsReAssoc);

#endif /* !_IE_SORT_H */

