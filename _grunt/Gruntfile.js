module.exports = function (grunt) {
  grunt.initConfig({
    bower: {
      install: {
        options: {
          targetDir: "../lib",
          cleanup: true,
          layout: "byComponent",
          verbose: true
        }
      }
    }
  });
  
  grunt.loadNpmTasks('grunt-bower-task');

  grunt.registerTask('default', ['bower:install']);
};
