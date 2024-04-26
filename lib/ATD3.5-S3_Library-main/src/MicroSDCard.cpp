#include "Common.h"
#include "MicroSDCard.h"
#include "vfs_api.h"
#include "sd_diskio.h"
#include "ff.h"
#include "FS.h"

static const char * TAG = "Card";
#ifndef SD_CARD_DEBUG
#undef ESP_LOGV
#define ESP_LOGV(TAG, V, ...) ;
#endif

MicroSDCard::MicroSDCard(FSImplPtr impl): SDFS(impl) {}

bool MicroSDCard::begin() {
    pinMode(SD_CD_PIN, INPUT_PULLUP);

    if (digitalRead(SD_CD_PIN)) { // CD = HIGH => Card not found
        ESP_LOGE(TAG, "card not insert");
        return false;
    }

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(50);

    bool ok = SDFS::begin((uint8_t) SD_CS_PIN, SPI);
    if (!ok) {
        ESP_LOGE(TAG, "init card fail");
    }

    return ok;
}

#ifdef USE_LVGL
static bool ready_cb(lv_fs_drv_t *drv) {
    MicroSDCard * card = (MicroSDCard *) drv->user_data;

    if (digitalRead(SD_CD_PIN)) { // CD = HIGH => Card not found
        ESP_LOGE(TAG, "card not insert");
        return false;
    }

    if (card->cardType() == CARD_NONE) { // Check card type if unknow type then...
        return card->begin(); // Init card again
    }

    return true; // Anythings OK
}

static void * fs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
    MicroSDCard * card = (MicroSDCard *) drv->user_data;

    ESP_LOGV(TAG, "start open file at %s mode %s", path, mode == LV_FS_MODE_RD ? FILE_READ : FILE_WRITE);

    File * fp = new File;
    if (!fp) {
        ESP_LOGE(TAG, "malloc fail");
        return NULL;
    }

    *fp = card->open(path, mode == LV_FS_MODE_RD ? FILE_READ : FILE_WRITE);
    if (!*fp) {
        ESP_LOGW(TAG, "open file at %s fail", path);
        delete fp;
        return NULL;
    }
    
    return fp;
}

static lv_fs_res_t fs_close(lv_fs_drv_t *drv, void *file_p) {
    File * fp = (File *) file_p;
    ESP_LOGV(TAG, "close file %s", fp->name());
    fp->close();
    delete fp;
    return LV_FS_RES_OK;
}

static lv_fs_res_t fs_read(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) {
    File * fp = (File *) file_p;
    ESP_LOGV(TAG, "read file %s length %d bytes", fp->name(), btr);
    int i = fp->available();
    ESP_LOGV(TAG, "wait to read %d", i);
    if (fp->available()) {
        *br = fp->read((uint8_t *) buf, btr);
    } else {
        *br = 0;
    }
    
    if (*br != btr) {
        ESP_LOGW(TAG, "read file size %d but can read only %d bytes", btr, *br);
    }

    return LV_FS_RES_OK;
}

static lv_fs_res_t fs_write(lv_fs_drv_t *drv, void *file_p, const void *buf, uint32_t btw, uint32_t *bw) {
    File * fp = (File *) file_p;
    ESP_LOGV(TAG, "write file %s length %d bytes", fp->name(), btw);
    *bw = fp->write((uint8_t *) buf, btw);
    if (*bw != btw) {
        ESP_LOGW(TAG, "write file size %d but can write only %d bytes", btw, *bw);
    }

    return LV_FS_RES_OK;
}

static lv_fs_res_t fs_seek(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence) {
    File * fp = (File *) file_p;
    ESP_LOGV(TAG, "seek file %s to position %d from %d", fp->name(), pos, (int) whence);

    bool ok = false;
    if (whence == LV_FS_SEEK_SET) {
        ok = fp->seek(pos, SeekSet);
    } else if (whence == LV_FS_SEEK_CUR) {
        ok = fp->seek(pos, SeekCur);
    } else if (whence == LV_FS_SEEK_END) {
        ok = fp->seek(pos, SeekEnd);
    }
    ESP_LOGV(TAG, "seek file ok");

    return ok ? LV_FS_RES_OK : LV_FS_RES_HW_ERR;
}

static lv_fs_res_t fs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
    File * fp = (File *) file_p;
    *pos_p = fp->position();
    ESP_LOGV(TAG, "tell file %s is position %d", *pos_p);

    return LV_FS_RES_OK;
}

static void * fs_dir_open(lv_fs_drv_t *drv, const char *path) {
    MicroSDCard * card = (MicroSDCard *) drv->user_data;

    File * fp = (File *) lv_mem_alloc(sizeof(File));
    if (!fp) {
      return NULL;
    }

    *fp = card->open(path);
    if (!*fp) {
       lv_mem_free(fp);
       return NULL;
    }
    if (!fp->isDirectory()) {
        fp->close();
        lv_mem_free(fp);
        return NULL;
    }

    return fp;
}

static lv_fs_res_t fs_dir_read(lv_fs_drv_t *drv, void *rddir_p, char *fn) {
    File * dir = (File *) rddir_p;
    File entry =  dir->openNextFile();
    strcpy(fn, entry.name());

    return LV_FS_RES_OK;
}

static lv_fs_res_t fs_dir_close(lv_fs_drv_t *drv, void *rddir_p) {
    File * dir = (File *) rddir_p;
    dir->close();
    lv_mem_free(dir);
    return LV_FS_RES_OK;
}

void MicroSDCard::useLVGL() {
    lv_fs_drv_init(&this->drv);

    drv.letter = 'S';                      // An uppercase letter to identify the drive
    drv.cache_size = 0;                    // Cache size for reading in bytes. 0 to not cache.*/

    drv.ready_cb = ready_cb;               // Callback to tell if the drive is ready to use
    drv.open_cb = fs_open;                 // Callback to open a file
    drv.close_cb = fs_close;               // Callback to close a file
    drv.read_cb = fs_read;                 // Callback to read a file
    drv.write_cb = fs_write;               // Callback to write a file
    drv.seek_cb = fs_seek;                 // Callback to seek in a file (Move cursor)
    drv.tell_cb = fs_tell;                 // Callback to tell the cursor position 

    drv.dir_open_cb = fs_dir_open;         // Callback to open directory to read its content
    drv.dir_read_cb = fs_dir_read;         // Callback to read a directory's content
    drv.dir_close_cb = fs_dir_close;       // Callback to close a directory

    drv.user_data = this;                  // Any custom data if required*/

    lv_fs_drv_register(&drv);              // Finally register the drive*/
}
#endif

MicroSDCard Card = MicroSDCard(FSImplPtr(new VFSImpl()));

// LVGL extra
typedef struct {
    char * path;
    lv_img_dsc_t dsc;
    void * next;
} lv_file_cache_item_t;

static lv_file_cache_item_t * first_file_cache_item = NULL;

void lv_img_set_src_from_sd_card(lv_obj_t * obj, const char * path) {
    ESP_LOGV(TAG, "cache scan %s", path);
    { // Find file in cache
        lv_file_cache_item_t * item = first_file_cache_item;
        while(item) {
            if (strcmp(item->path, path) == 0) {
                ESP_LOGV(TAG, "found %s in cache", path);
                lv_img_set_src(obj, &item->dsc);
                return;
            }
            item = (lv_file_cache_item_t *) item->next;
        }
    }

    // Load file from SD Card to PSRAM
    ESP_LOGV(TAG, "start open file at %s", path);
    File f = Card.open(path);
    if (!f) {
        ESP_LOGE(TAG, "open file %s fail", path);
        return;
    }
    
    ESP_LOGV(TAG, "malloc cache file %s", path);
    lv_file_cache_item_t * cache_info = (lv_file_cache_item_t *) malloc(sizeof(lv_file_cache_item_t));
    if (!cache_info) {
        ESP_LOGE(TAG, "malloc cache file %s fail", path);
        f.close();
        return;
    }
    memset(cache_info, 0, sizeof(lv_file_cache_item_t));

    { // Copy path to cache
        ESP_LOGV(TAG, "malloc path cache file %s", path);
        size_t path_len = strlen(path);
        cache_info->path = (char *) malloc(path_len + 1);
        if (!cache_info->path) {
            ESP_LOGE(TAG, "malloc cache path file %s fail", path);
            free(cache_info);
            f.close();
            return;
        }
        memset(cache_info->path, 0, path_len + 1);
        strcpy(cache_info->path, path);
    }

    ESP_LOGV(TAG, "malloc buffer file %s", path);
    size_t file_size = f.size();
    uint8_t * buff = (uint8_t *) ps_malloc(file_size);
    if (!buff) {
        ESP_LOGE(TAG, "malloc file %s fail", path);
        free(cache_info->path);
        free(cache_info);
        f.close();
        return;
    }

    ESP_LOGV(TAG, "read file %s length %d bytes", f.name(), file_size);
    size_t byte_read = f.readBytes((char *) buff, file_size);
    if (byte_read != file_size) {
        ESP_LOGE(TAG, "read file size %d but can read only %d bytes", file_size, byte_read);
        free(buff);
        free(cache_info->path);
        free(cache_info);
        f.close();
        return;
    }

    ESP_LOGV(TAG, "close file %s", f.name());
    f.close();

    cache_info->dsc.header.always_zero = 0;
    cache_info->dsc.header.w = 0; // Auto width after decode
    cache_info->dsc.header.h = 0; // Auto hight after decode
    cache_info->dsc.header.cf = 0; // Auto color format after decode
    cache_info->dsc.data_size = file_size;
    cache_info->dsc.data = buff;
    lv_img_set_src(obj, &cache_info->dsc);

    { // Add to list
        if (!first_file_cache_item) {
            first_file_cache_item = cache_info;
        } else {
            lv_file_cache_item_t * item = first_file_cache_item;
            while(item) {
                if (!item->next) {
                    item->next = (void *) cache_info;
                    break;
                }
                item = (lv_file_cache_item_t * ) item->next;
            }
        }
    }
}

