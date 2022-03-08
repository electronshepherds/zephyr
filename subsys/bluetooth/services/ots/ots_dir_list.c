/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/dlist.h>
#include <sys/byteorder.h>

#include <bluetooth/services/ots.h>
#include "ots_internal.h"
#include "ots_obj_manager_internal.h"
#include "ots_dir_list_internal.h"

#include <logging/log.h>

LOG_MODULE_DECLARE(bt_ots, CONFIG_BT_OTS_LOG_LEVEL);

/* In order to transmit the maximum size we choose CONFIG_BT_OTS_L2CAP_CHAN_TX_MTU
 * as the buffer size. However, directory listing encoding can only happen at
 * directory entry boundary so we must ensure the MTU value is rounded up to the
 * nearest directory entry size to not overflow our buffer. Also since we can only
 * encode at object entry boundary, we need to make sure we have at least room for
 * two entries so entries can be read from the middle.
 */
#define OTS_DIR_LIST_LAZY_BUFFER_SIZE MAX(2 * DIR_LIST_OBJ_RECORD_MAX_SIZE, \
	DIR_LIST_OBJ_RECORD_MAX_SIZE * \
	ceiling_fraction(CONFIG_BT_OTS_L2CAP_CHAN_TX_MTU, DIR_LIST_OBJ_RECORD_MAX_SIZE))

struct bt_ots_dir_list {
	struct net_buf_simple net_buf;
	struct bt_gatt_ots_object *dir_list_obj;
	uint8_t _content[OTS_DIR_LIST_LAZY_BUFFER_SIZE];
};

static struct bt_ots_dir_list dir_lists[CONFIG_BT_OTS_MAX_INST_CNT];

static size_t dir_list_object_encode(const struct bt_gatt_ots_object *obj,
				     struct net_buf_simple *net_buf)
{
	uint8_t flags = 0;
	uint8_t *start = NULL;
	uint16_t len;
	size_t obj_name_len;

	BT_OTS_DIR_LIST_SET_FLAG_PROPERTIES(flags);
	BT_OTS_DIR_LIST_SET_FLAG_CUR_SIZE(flags);
	if (obj->metadata.type.uuid.type == BT_UUID_TYPE_128) {
		BT_OTS_DIR_LIST_SET_FLAG_TYPE_128(flags);
	}

	/* skip 16bits at the beginning of the record for the record's length */
	if (net_buf != NULL) {
		start = net_buf_simple_add(net_buf, sizeof(len));
	}
	len = sizeof(len);

	/* ID */
	if (net_buf != NULL) {
		net_buf_simple_add_le48(net_buf, obj->id);
	}
	len += BT_OTS_OBJ_ID_SIZE;

	/* Name length */
	obj_name_len = strlen(obj->metadata.name);
	__ASSERT(obj_name_len > 0 && obj_name_len <= CONFIG_BT_OTS_OBJ_MAX_NAME_LEN,
		 "Dir list object len is incorrect %zu", obj_name_len);
	if (net_buf != NULL) {
		net_buf_simple_add_u8(net_buf, obj_name_len);
	}
	len += sizeof(uint8_t);

	/* Name */
	if (net_buf != NULL) {
		net_buf_simple_add_mem(net_buf, obj->metadata.name, obj_name_len);
	}
	len += obj_name_len;

	/* Flags */
	if (net_buf != NULL) {
		net_buf_simple_add_u8(net_buf, flags);
	}
	len += sizeof(flags);

	/* encode Object type */
	if (obj->metadata.type.uuid.type == BT_UUID_TYPE_16) {
		if (net_buf != NULL) {
			net_buf_simple_add_le16(net_buf, obj->metadata.type.uuid_16.val);
		}
		len += BT_UUID_SIZE_16;
	} else {
		if (net_buf != NULL) {
			net_buf_simple_add_mem(net_buf, obj->metadata.type.uuid_128.val,
					BT_UUID_SIZE_128);
		}
		len += BT_UUID_SIZE_128;
	}

	/* encode Object Current size */
	if (net_buf != NULL) {
		net_buf_simple_add_le32(net_buf, obj->metadata.size.cur);
	}
	len += sizeof(obj->metadata.size.cur);

	/* Object properties */
	if (net_buf != NULL) {
		net_buf_simple_add_le32(net_buf, obj->metadata.props);
	}
	len += sizeof(obj->metadata.props);

	__ASSERT(len >= DIR_LIST_OBJ_RECORD_MIN_SIZE,
		 "Dir list object len is too small %u", len);
	__ASSERT(len <= DIR_LIST_OBJ_RECORD_MAX_SIZE,
		 "Dir list object len is too large %u", len);

	/* Update the record length at the beginning */
	if (start != NULL) {
		sys_put_le16(len, start);
	}

	return len;
}

static void dir_list_update_size(struct bt_ots_dir_list *dir_list, void *obj_manager)
{
	struct bt_gatt_ots_object *obj;
	int err;
	size_t len = 0;

	err = bt_gatt_ots_obj_manager_first_obj_get(obj_manager, &obj);

	__ASSERT(err == 0 && obj == dir_list->dir_list_obj,
		 "Expecting first object to be the Directory Listing Object");

	do {
		len += dir_list_object_encode(obj, NULL);

		err = bt_gatt_ots_obj_manager_next_obj_get(obj_manager, obj, &obj);
	} while (!err);

	/* Update Directory Listing Object metadata size */
	dir_list->dir_list_obj->metadata.size.cur = len;
}

void bt_ots_dir_list_selected(struct bt_ots_dir_list *dir_list, void *obj_manager,
			      struct bt_gatt_ots_object *cur_obj)
{
	if (dir_list->dir_list_obj != cur_obj) {
		/* We only need to update the object directory listing if it is currently selected,
		 * as we otherwise only create it when it is selected.
		 */
		return;
	}

	dir_list_update_size(dir_list, obj_manager);
}

void bt_ots_dir_list_init(struct bt_ots_dir_list **dir_list, void *obj_manager)
{
	struct bt_gatt_ots_object *dir_list_obj;
	int err;
	static char *dir_list_obj_name = CONFIG_BT_OTS_DIR_LIST_OBJ_NAME;

	__ASSERT(*dir_list == NULL, "Already initialized");

	for (size_t i = 0; i < ARRAY_SIZE(dir_lists); i++) {
		if (!dir_lists[i].dir_list_obj) {
			*dir_list = &dir_lists[i];
		}
	}

	__ASSERT(*dir_list, "Could not assign Directory Listing Object");

	__ASSERT(strlen(dir_list_obj_name) <= CONFIG_BT_OTS_OBJ_MAX_NAME_LEN,
		 "BT_OTS_DIR_LIST_OBJ_NAME shall be less than or equal to %u octets",
		 CONFIG_BT_OTS_OBJ_MAX_NAME_LEN);

	err = bt_gatt_ots_obj_manager_obj_add(obj_manager, &dir_list_obj);

	__ASSERT(!err, "Could not add Directory Listing Object for object manager %p", obj_manager);

	memset(&dir_list_obj->metadata, 0, sizeof(dir_list_obj->metadata));
	dir_list_obj->metadata.name = dir_list_obj_name;
	dir_list_obj->metadata.size.alloc = DIR_LIST_MAX_SIZE;
	dir_list_obj->metadata.type.uuid.type = BT_UUID_TYPE_16;
	dir_list_obj->metadata.type.uuid_16.val = BT_UUID_OTS_DIRECTORY_LISTING_VAL;
	BT_OTS_OBJ_SET_PROP_READ(dir_list_obj->metadata.props);

	(*dir_list)->dir_list_obj = dir_list_obj;

	dir_list_update_size(*dir_list, obj_manager);
}

ssize_t bt_ots_dir_list_content_get(struct bt_ots_dir_list *dir_list, void *obj_manager,
				    void **data, size_t len, off_t offset)
{
	struct bt_gatt_ots_object *obj;
	int err;
	size_t buffer_offset = 0;
	size_t rec_len = 0;
	size_t room = 0;
	bool found = false;

	err = bt_gatt_ots_obj_manager_first_obj_get(obj_manager, &obj);

	__ASSERT(err == 0 && obj == dir_list->dir_list_obj,
		 "Expecting first object to be the Directory Listing Object");

	do {
		if (!found) {
			buffer_offset += rec_len;

			net_buf_simple_init_with_data(&dir_list->net_buf,
						      dir_list->_content,
						      sizeof(dir_list->_content));
			net_buf_simple_reset(&dir_list->net_buf);
			room = sizeof(dir_list->_content);
		}

		rec_len = dir_list_object_encode(obj, &dir_list->net_buf);

		if (!found && buffer_offset + rec_len > offset) {
			found = true;
		}

		room -= rec_len;

		err = bt_gatt_ots_obj_manager_next_obj_get(obj_manager, obj, &obj);
	} while (!err && room >= DIR_LIST_OBJ_RECORD_MAX_SIZE);

	*data = dir_list->net_buf.data + (offset - buffer_offset);

	return MIN(len, dir_list->net_buf.len - (offset - buffer_offset));
}

bool bt_ots_dir_list_is_idle(const struct bt_ots_dir_list *dir_list)
{
	return dir_list->dir_list_obj->state.type == BT_GATT_OTS_OBJECT_IDLE_STATE;
}
