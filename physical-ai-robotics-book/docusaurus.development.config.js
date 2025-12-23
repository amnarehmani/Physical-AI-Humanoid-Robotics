// @ts-check

/**
 * Webpack + HTML injection config
 */

const path = require('path');

/** @type {import('@docusaurus/types').PluginModule} */
module.exports = {
  /**
   * @param {import('webpack').Configuration} config
   * @param {boolean} isServer
   * @returns {import('webpack').Configuration}
   */
  configureWebpack(config, isServer) {
    if (!isServer) {
      config.resolve = config.resolve || {};
      config.resolve.fallback = {
        ...(config.resolve.fallback || {}),
        fs: false,
        path: false,
      };
    }
    return config;
  },

  /**
   * Inject HTML tags (Docusaurus expects an ARRAY)
   * @returns {(string | import('@docusaurus/types').HtmlTagObject)[]}
   */
  injectHtmlTags() {
    return [];
  },
};
