#!/bin/sh
rclone delete gdrive:fotogramas_rpi
rclone copy build/images/  gdrive:/fotogramas_rpi
