//load home page

var express = require('express');
var router = express.Router();

/* GET home page. */
router.all('/', function(req, res) {
  /*var title = req.body.title;
  var bodyy = req.body.bodyy;
  var userId = req.body.userId;
  console.log('test');
  console.log(req.body);
  res.send('title is: ' + title + ' and bodyy is: ' + bodyy + ' and userId is: ' + userId);*/
  //console.log('received GET from ESP');
  res.render('index', { title: 'Express' });
  
});

module.exports = router;
