# Security Policy

## Security Updates

### Latest Security Patches

**December 2024**
- Updated `fastapi` from 0.109.0 to 0.109.1 (fixes Content-Type Header ReDoS vulnerability)
- Updated `python-multipart` from 0.0.6 to 0.0.18 (fixes multiple vulnerabilities):
  - DoS via deformation multipart/form-data boundary
  - Content-Type Header ReDoS

All dependencies are now on patched versions with no known vulnerabilities.

## Supported Versions

This project is currently in active development. Security updates will be applied to the main branch.

| Version | Supported          |
| ------- | ------------------ |
| main    | :white_check_mark: |
| develop | :white_check_mark: |

## Reporting a Vulnerability

If you discover a security vulnerability, please report it by:

1. **Do NOT** open a public issue
2. Email the maintainer directly at the contact provided in the repository
3. Include a detailed description of the vulnerability
4. Provide steps to reproduce if possible

We aim to respond to security reports within 48 hours.

## Security Best Practices

### API Key Management

- **Never commit API keys** to version control
- Use environment variables for all sensitive data
- Keep your `.env` file in `.gitignore`
- Rotate API keys regularly
- Use separate API keys for development and production

### Network Security

- **Use HTTPS/WSS** in production environments
- Implement proper firewall rules on all devices
- Restrict WebSocket connections to trusted clients
- Use VPN for remote access to Raspberry Pi

### Authentication & Authorization

- Implement authentication for production deployments
- Use strong, unique passwords for all services
- Enable two-factor authentication where possible
- Regularly review and revoke unused access

### Hardware Security

- **Physical Access**: Secure physical access to Raspberry Pi and ESP12E
- **Network Isolation**: Consider isolating robot network from main network
- **Update Firmware**: Keep all firmware and software up to date
- **Disable Unused Services**: Disable SSH, Bluetooth, etc. if not needed

### Code Security

- **Input Validation**: Always validate user inputs
- **Dependency Updates**: Regularly update dependencies
- **Code Review**: Review all code changes
- **Static Analysis**: Use security linters and scanners

## Security Features Implemented

### Environment Variable Protection
- All sensitive configuration uses environment variables
- Example `.env.example` file provided without real credentials
- `.gitignore` configured to exclude `.env` files

### Logging
- Comprehensive logging for debugging and security monitoring
- Log levels configurable via environment variables
- Sensitive data not logged

### CORS Configuration
- CORS middleware configured (set to `*` for development)
- Should be restricted to specific origins in production

### Error Handling
- Graceful error handling prevents information leakage
- Generic error messages to clients
- Detailed errors only in server logs

## Production Deployment Recommendations

### Before Deploying to Production

1. **Environment Setup**
   ```bash
   # Set secure environment variables
   export GEMINI_API_KEY="your-production-key"
   export LOG_LEVEL="WARNING"
   ```

2. **CORS Configuration**
   Update `server/server.py`:
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["https://yourdomain.com"],  # Specific domain
       allow_credentials=True,
       allow_methods=["GET", "POST"],
       allow_headers=["*"],
   )
   ```

3. **HTTPS/WSS Setup**
   - Use a reverse proxy (nginx) with SSL certificates
   - Use Let's Encrypt for free SSL certificates
   - Update WebSocket URLs to use `wss://`

4. **Authentication**
   - Implement token-based authentication
   - Add authentication middleware
   - Use OAuth2 or JWT for API access

5. **Rate Limiting**
   - Implement rate limiting on API endpoints
   - Prevent DoS attacks
   - Use libraries like `slowapi`

6. **Monitoring**
   - Set up log monitoring
   - Configure alerts for suspicious activity
   - Monitor resource usage

7. **Backups**
   - Regular backups of configuration
   - Version control for all code
   - Document disaster recovery procedures

### Firewall Configuration

#### Raspberry Pi
```bash
# Allow only necessary ports
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow from 192.168.1.0/24 to any port 22  # SSH from local network only
sudo ufw enable
```

#### Server
```bash
# Allow HTTP/HTTPS and WebSocket
sudo ufw allow 80/tcp
sudo ufw allow 443/tcp
sudo ufw allow 8000/tcp  # For development only
sudo ufw enable
```

## Security Checklist for Production

- [ ] All API keys stored in environment variables
- [ ] `.env` file excluded from version control
- [ ] CORS restricted to specific domains
- [ ] HTTPS/WSS enabled
- [ ] Authentication implemented
- [ ] Rate limiting configured
- [ ] Firewall rules configured
- [ ] Regular security updates scheduled
- [ ] Monitoring and alerting configured
- [ ] Backup procedures documented
- [ ] Incident response plan created
- [ ] Security audit completed

## Known Security Considerations

1. **Default CORS Settings**: The application currently allows all origins (`*`). This should be restricted in production.

2. **No Authentication**: There is no authentication on WebSocket or API endpoints. Implement before production use.

3. **API Key Storage**: Ensure `.env` file permissions are restricted (`chmod 600 .env`).

4. **Network Exposure**: Minimize network exposure of Raspberry Pi and ESP12E.

5. **Input Validation**: While basic validation exists, comprehensive input sanitization should be reviewed.

## Resources

- [OWASP Top Ten](https://owasp.org/www-project-top-ten/)
- [FastAPI Security](https://fastapi.tiangolo.com/tutorial/security/)
- [Raspberry Pi Security](https://www.raspberrypi.org/documentation/configuration/security.md)
- [IoT Security Best Practices](https://www.iotsecurityfoundation.org/best-practice-guidelines/)

## Updates

This security policy will be updated as new features are added and security practices evolve.

Last Updated: December 2024
