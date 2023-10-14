document.write('<link rel="stylesheet" href="https://github.githubassets.com/assets/gist-embed-0e9bf67cc0c6.css">')
document.write('<div id=\"gist99864333\" class=\"gist\">\n    <div class=\"gist-file\" translate=\"no\">\n      <div class=\"gist-data\">\n        <div class=\"js-gist-file-update-container js-task-list-container file-box\">\n  <div id=\"file-geocentrilatitude-js\" class=\"file my-2\">\n    \n    <div itemprop=\"text\" class=\"Box-body p-0 blob-wrapper data type-javascript  \">\n\n        \n<div class=\"js-check-bidi js-blob-code-container blob-code-content\">\n\n  <template class=\"js-file-alert-template\">\n  <div data-view-component=\"true\" class=\"flash flash-warn flash-full d-flex flex-items-center\">\n  <svg aria-hidden=\"true\" height=\"16\" viewBox=\"0 0 16 16\" version=\"1.1\" width=\"16\" data-view-component=\"true\" class=\"octicon octicon-alert\">\n    <path d=\"M6.457 1.047c.659-1.234 2.427-1.234 3.086 0l6.082 11.378A1.75 1.75 0 0 1 14.082 15H1.918a1.75 1.75 0 0 1-1.543-2.575Zm1.763.707a.25.25 0 0 0-.44 0L1.698 13.132a.25.25 0 0 0 .22.368h12.164a.25.25 0 0 0 .22-.368Zm.53 3.996v2.5a.75.75 0 0 1-1.5 0v-2.5a.75.75 0 0 1 1.5 0ZM9 11a1 1 0 1 1-2 0 1 1 0 0 1 2 0Z\"><\/path>\n<\/svg>\n    <span>\n      This file contains bidirectional Unicode text that may be interpreted or compiled differently than what appears below. To review, open the file in an editor that reveals hidden Unicode characters.\n      <a class=\"Link--inTextBlock\" href=\"https://github.co/hiddenchars\" target=\"_blank\">Learn more about bidirectional Unicode characters<\/a>\n    <\/span>\n\n\n  <div data-view-component=\"true\" class=\"flash-action\">        <a href=\"{{ revealButtonHref }}\" data-view-component=\"true\" class=\"btn-sm btn\">    Show hidden characters\n<\/a>\n<\/div>\n<\/div><\/template>\n<template class=\"js-line-alert-template\">\n  <span aria-label=\"This line has hidden Unicode characters\" data-view-component=\"true\" class=\"line-alert tooltipped tooltipped-e\">\n    <svg aria-hidden=\"true\" height=\"16\" viewBox=\"0 0 16 16\" version=\"1.1\" width=\"16\" data-view-component=\"true\" class=\"octicon octicon-alert\">\n    <path d=\"M6.457 1.047c.659-1.234 2.427-1.234 3.086 0l6.082 11.378A1.75 1.75 0 0 1 14.082 15H1.918a1.75 1.75 0 0 1-1.543-2.575Zm1.763.707a.25.25 0 0 0-.44 0L1.698 13.132a.25.25 0 0 0 .22.368h12.164a.25.25 0 0 0 .22-.368Zm.53 3.996v2.5a.75.75 0 0 1-1.5 0v-2.5a.75.75 0 0 1 1.5 0ZM9 11a1 1 0 1 1-2 0 1 1 0 0 1 2 0Z\"><\/path>\n<\/svg>\n<\/span><\/template>\n\n  <table data-hpc class=\"highlight tab-size js-file-line-container js-code-nav-container js-tagsearch-file\" data-tab-size=\"8\" data-paste-markdown-skip data-tagsearch-lang=\"JavaScript\" data-tagsearch-path=\"GeocentriLatitude.js\">\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L1\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"1\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC1\" class=\"blob-code blob-code-inner js-file-line\"><span class=pl-k>function<\/span> <span class=pl-v>GeocentricLatitude<\/span><span class=pl-kos>(<\/span><span class=pl-s1>lat<\/span><span class=pl-kos>)<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L2\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"2\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC2\" class=\"blob-code blob-code-inner js-file-line\"><span class=pl-kos>{<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L3\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"3\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC3\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-c>// Convert geodetic latitude &#39;lat&#39; to a geocentric latitude &#39;clat&#39;.<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L4\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"4\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC4\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-c>// Geodetic latitude is the latitude as given by GPS.<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L5\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"5\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC5\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-c>// Geocentric latitude is the angle measured from center of Earth between a point and the equator.<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L6\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"6\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC6\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-c>// https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L7\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"7\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC7\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-k>var<\/span> <span class=pl-s1>e2<\/span> <span class=pl-c1>=<\/span> <span class=pl-c1>0.00669437999014<\/span><span class=pl-kos>;<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L8\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"8\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC8\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-k>var<\/span> <span class=pl-s1>clat<\/span> <span class=pl-c1>=<\/span> <span class=pl-v>Math<\/span><span class=pl-kos>.<\/span><span class=pl-en>atan<\/span><span class=pl-kos>(<\/span><span class=pl-kos>(<\/span><span class=pl-c1>1.0<\/span> <span class=pl-c1>-<\/span> <span class=pl-s1>e2<\/span><span class=pl-kos>)<\/span> <span class=pl-c1>*<\/span> <span class=pl-v>Math<\/span><span class=pl-kos>.<\/span><span class=pl-en>tan<\/span><span class=pl-kos>(<\/span><span class=pl-s1>lat<\/span><span class=pl-kos>)<\/span><span class=pl-kos>)<\/span><span class=pl-kos>;<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L9\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"9\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC9\" class=\"blob-code blob-code-inner js-file-line\">    <span class=pl-k>return<\/span> <span class=pl-s1>clat<\/span><span class=pl-kos>;<\/span><\/td>\n        <\/tr>\n        <tr>\n          <td id=\"file-geocentrilatitude-js-L10\" class=\"blob-num js-line-number js-code-nav-line-number js-blob-rnum\" data-line-number=\"10\"><\/td>\n          <td id=\"file-geocentrilatitude-js-LC10\" class=\"blob-code blob-code-inner js-file-line\"><span class=pl-kos>}<\/span><\/td>\n        <\/tr>\n  <\/table>\n<\/div>\n\n\n    <\/div>\n\n  <\/div>\n<\/div>\n\n      <\/div>\n      <div class=\"gist-meta\">\n        <a href=\"https://gist.github.com/cosinekitty/6c66dc21bb0d9df9c3bec202115d7cbe/raw/9ff3d9c37b974959b3d6064d431dca3dda1f15f9/GeocentriLatitude.js\" style=\"float:right\">view raw<\/a>\n        <a href=\"https://gist.github.com/cosinekitty/6c66dc21bb0d9df9c3bec202115d7cbe#file-geocentrilatitude-js\">\n          GeocentriLatitude.js\n        <\/a>\n        hosted with &#10084; by <a class=\"Link--inTextBlock\" href=\"https://github.com\">GitHub<\/a>\n      <\/div>\n    <\/div>\n<\/div>\n')
