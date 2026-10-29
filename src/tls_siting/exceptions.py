"""Custom exceptions and warnings for TLS Siting Engine."""


class SitingError(Exception):
    """Base exception class for TLS siting errors."""
    pass


class PhysicalInvariantError(SitingError):
    """Exception raised for non-negotiable physical invariant breaches."""
    pass


class RegulatoryWarning(UserWarning):
    """Warning raised for configurable constraint violations when waivers are allowed."""
    pass
